#!/usr/bin/env python3

import math
import struct
from typing import List

import rclpy
from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState

# XL330 (X-Series, Protocol 2.0) — kaair_driver/head 와 동일
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

DXL_OFFSET_DEFAULT = 2048
DXL_RESOLUTION = 4096.0


def _unsigned32_to_signed32(u: int) -> int:
    u &= 0xFFFFFFFF
    if u >= 0x80000000:
        return int(u - 0x100000000)
    return int(u)


class MasterDxlHw(Node):
    """8축 Dynamixel XL330: Sync Read 현재값 → JointState 퍼블리시, 명령 토픽으로 Sync Write."""

    def __init__(self):
        super().__init__('master_dxl_hw')

        self.declare_parameter('device_name', '/dev/ttyMaster')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6, 7, 8])
        self.declare_parameter(
            'joint_names',
            [
                'master_joint_1',
                'master_joint_2',
                'master_joint_3',
                'master_joint_4',
                'master_joint_5',
                'master_joint_6',
                'master_joint_7',
                'master_joint_8',
            ],
        )
        self.declare_parameter('publish_topic', '/master/joint_states')
        self.declare_parameter('command_topic', '/master/joint_commands')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('torque_enable_on_start', True)
        # motor_ids 와 동일한 순서. 비어 있으면 dxl_offset_default 로 전 축 동일 적용
        self.declare_parameter('motor_position_offsets', [2048, 2048, 2048, 2048, 4096, 2048, 2048, 2048])
        self.declare_parameter('dxl_offset_default', DXL_OFFSET_DEFAULT)
        # XL330 current-based position control 모드에서 목표 전류를 축별로 지정
        self.declare_parameter('motor_goal_currents', [90,10,10,10,10,10,10,1750])  # motor_ids 와 동일한 순서, int16(단위는 제어기/펌웨어 설정에 따름)
        # 초기화 시 1회만 적용할 초기 자세(rad)
        self.declare_parameter('initial_pose', [0.0, 0.261799, 0.0, 0.261799, -3.14159, 1.5708, 0.0, 0.0])

        device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._protocol = float(self.get_parameter('protocol_version').value)
        self._ids: List[int] = list(self.get_parameter('motor_ids').get_parameter_value().integer_array_value)
        self._joint_names: List[str] = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)

        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        publish_hz = float(self.get_parameter('publish_hz').value)
        self._torque_on_start = self.get_parameter('torque_enable_on_start').get_parameter_value().bool_value

        offsets_param = list(
            self.get_parameter('motor_position_offsets').get_parameter_value().integer_array_value
        )
        dxl_off_default = int(
            self.get_parameter('dxl_offset_default').get_parameter_value().integer_value
        )

        if len(self._ids) != len(self._joint_names):
            raise ValueError(
                f'motor_ids({len(self._ids)}) 와 joint_names({len(self._joint_names)}) 개수가 같아야 함'
            )

        self._num = len(self._ids)

        if not offsets_param:
            self._motor_offsets = [dxl_off_default] * self._num
        elif len(offsets_param) != self._num:
            raise ValueError(
                f'motor_position_offsets 길이({len(offsets_param)}) 가 motor 수({self._num}) 와 같아야 함 '
                '(또는 빈 리스트로 dxl_offset_default 사용)'
            )
        else:
            self._motor_offsets = [int(o) for o in offsets_param]

        goal_currents_param = list(
            self.get_parameter('motor_goal_currents').get_parameter_value().integer_array_value
        )
        initial_pose_param = list(
            self.get_parameter('initial_pose').get_parameter_value().double_array_value
        )
        if not goal_currents_param:
            raise ValueError(
                'motor_goal_currents 가 비어있음 — motor_ids 와 같은 길이의 int 배열로 설정 필요'
            )
        elif len(goal_currents_param) != self._num:
            raise ValueError(
                f'motor_goal_currents 길이({len(goal_currents_param)}) 가 motor 수({self._num}) 와 같아야 함 '
                '(motor_ids 와 동일 길이로 설정 필요)'
            )
        else:
            self._motor_goal_currents = [int(c) for c in goal_currents_param]

        if len(initial_pose_param) != self._num:
            raise ValueError(
                f'initial_pose 길이({len(initial_pose_param)}) 가 motor 수({self._num}) 와 같아야 함'
            )
        self._initial_pose = [float(v) for v in initial_pose_param]

        self._port_handler = PortHandler(device_name)
        self._packet_handler = PacketHandler(self._protocol)

        if not self._port_handler.openPort():
            raise RuntimeError(f'포트 오픈 실패: {device_name}')
        if not self._port_handler.setBaudRate(int(self._baudrate)):
            self._port_handler.closePort()
            raise RuntimeError(f'보드레이트 설정 실패: {self._baudrate}')

        if self._torque_on_start:
            self._set_all_torque(True)
        self._write_goal_current_once()
        self._write_initial_pose_once()

        self._gsr = GroupSyncRead(self._port_handler, self._packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in self._ids:
            if not self._gsr.addParam(int(dxl_id)):
                self._cleanup_hardware()
                raise RuntimeError(f'GroupSyncRead addParam 실패 (ID {dxl_id})')

        self._publisher = self.create_publisher(JointState, publish_topic, 10)
        self.create_subscription(JointState, command_topic, self._on_joint_command, 10)

        self.create_timer(max(1.0 / publish_hz, 0.002), self._tick)

        self.get_logger().info(
            f'{self._num}축 Dynamixel 시작: {device_name} @ {self._baudrate}, '
            f'pub={publish_topic} cmd={command_topic}, offsets={dict(zip(self._ids, self._motor_offsets))}, '
            f'goal_current={dict(zip(self._ids, self._motor_goal_currents))}'
        )

    def destroy_node(self):
        self._cleanup_hardware()
        super().destroy_node()

    def _cleanup_hardware(self):
        if getattr(self, '_port_handler', None) is None:
            return
        try:
            if self._torque_on_start:
                self._set_all_torque(False)
        except Exception:
            pass
        try:
            if self._port_handler.is_open:
                self._port_handler.closePort()
        except Exception:
            pass

    def _set_all_torque(self, enable: bool) -> None:
        val = 1 if enable else 0
        for dxl_id in self._ids:
            result, err = self._packet_handler.write1ByteTxRx(
                self._port_handler, int(dxl_id), ADDR_TORQUE_ENABLE, val
            )
            if result != COMM_SUCCESS:
                self.get_logger().error(
                    f'Torque enable 실패 ID {dxl_id}: '
                    f'{self._packet_handler.getTxRxResult(result)}'
                )
            elif err != 0:
                self.get_logger().error(
                    f'Torque 패킷 에러 ID {dxl_id}: '
                    f'{self._packet_handler.getRxPacketError(err)}'
                )

    def _rad_to_dxl_goal(self, rad: float, encoder_offset: int) -> int:
        v = int(round((rad * DXL_RESOLUTION / (2.0 * math.pi)) + encoder_offset))
        packed = struct.pack('i', v)
        return struct.unpack('I', packed)[0]

    def _write_goal_current_once(self) -> None:
        self.get_logger().info(
            f'init goal_current write: {dict(zip(self._ids, self._motor_goal_currents))}'
        )
        for dxl_id, goal_cur in zip(self._ids, self._motor_goal_currents):
            u16 = int(goal_cur) & 0xFFFF  # signed int16 -> unsigned word
            dxl_comm_result, dxl_error = self._packet_handler.write2ByteTxRx(
                self._port_handler, int(dxl_id), ADDR_GOAL_CURRENT, u16
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warning(
                    f'Goal current write 실패 ID {dxl_id}: '
                    f'{self._packet_handler.getTxRxResult(dxl_comm_result)}'
                )
            elif dxl_error != 0:
                self.get_logger().warning(
                    f'Goal current 패킷 에러 ID {dxl_id}: '
                    f'{self._packet_handler.getRxPacketError(dxl_error)}'
                )

    def _write_initial_pose_once(self) -> None:
        gsw = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        for dxl_id, rad, enc_off in zip(self._ids, self._initial_pose, self._motor_offsets):
            goal = self._rad_to_dxl_goal(rad, enc_off)
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)),
                DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)),
                DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            if not gsw.addParam(int(dxl_id), param):
                gsw.clearParam()
                raise RuntimeError(f'초기 자세 addParam 실패 (ID {dxl_id})')

        result = gsw.txPacket()
        gsw.clearParam()
        if result != COMM_SUCCESS:
            raise RuntimeError('초기 자세 Sync Write 실패: ' + self._packet_handler.getTxRxResult(result))

        self.get_logger().info(
            f'init pose write(rad): {dict(zip(self._ids, self._initial_pose))}'
        )

    def _dxl_present_to_rad(self, raw_unsigned: int, encoder_offset: int) -> float:
        s = _unsigned32_to_signed32(raw_unsigned)
        return (float(s) - encoder_offset) * (2.0 * math.pi / DXL_RESOLUTION)

    def _tick(self):
        dxl_comm = self._gsr.txRxPacket()
        if dxl_comm != COMM_SUCCESS:
            self.get_logger().warn(
                'Sync Read 실패: ' + self._packet_handler.getTxRxResult(dxl_comm),
                throttle_duration_sec=1.0,
            )
            return

        positions: List[float] = []
        for dxl_id, enc_off in zip(self._ids, self._motor_offsets):
            if not self._gsr.isAvailable(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                self.get_logger().warn(f'위치 데이터 없음 ID {dxl_id}', throttle_duration_sec=1.0)
                return
            raw = self._gsr.getData(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            positions.append(self._dxl_present_to_rad(int(raw), enc_off))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        self._publisher.publish(msg)

    def _on_joint_command(self, msg: JointState):
        """JointState.position: joint_names 과 동일 이름 순 또는 메시지 내 순서—기본적으로 파라미터 joint_names 순서로 매칭."""
        rad_list: List[float] = []

        if msg.name:
            idx = {n: i for i, n in enumerate(msg.name)}
            for name in self._joint_names:
                if name not in idx:
                    self.get_logger().warning(
                        f'명령에 joint "{name}" 없음 — 무시', throttle_duration_sec=2.0
                    )
                    return
                j = idx[name]
                if j >= len(msg.position):
                    return
                rad_list.append(float(msg.position[j]))
        else:
            if len(msg.position) != self._num:
                self.get_logger().warning(
                    f'이름 없는 명령은 position 길이가 {self._num} 이어야 함 '
                    f'(got {len(msg.position)})',
                    throttle_duration_sec=2.0,
                )
                return
            rad_list = [float(p) for p in msg.position]

        gsw = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        for dxl_id, rad, enc_off in zip(self._ids, rad_list, self._motor_offsets):
            goal = self._rad_to_dxl_goal(rad, enc_off)
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)),
                DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)),
                DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            if not gsw.addParam(int(dxl_id), param):
                gsw.clearParam()
                return

        if gsw.txPacket() != COMM_SUCCESS:
            self.get_logger().warn('Sync Write 실패', throttle_duration_sec=0.5)
        gsw.clearParam()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MasterDxlHw()
    except Exception as e:
        rclpy.logging.get_logger('master_dxl_hw').fatal(f'초기화 실패: {e}')
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
