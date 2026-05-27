#!/usr/bin/env python3

"""
xarm_bridge.py
`/arm/init_set`(std_srvs/Trigger) 수신 시 xarm Python SDK 로 컨트롤러와 직접 통신하여
clean_error → motion_enable → set_mode → set_state 순 초기화를 수행한다.
또한 주기적인 타이머를 활용하여 7셀 리튬폴리머 배터리의 전압 및 잔량을 모니터링한다.
"""

import os

_sdk_log_root = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'log', 'xarm', 'sdk')
try:
    os.makedirs(_sdk_log_root, exist_ok=True)
except OSError:
    pass

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from xarm.wrapper import XArmAPI


def _code_ok(code) -> bool:
    return code is None or code == 0


class XArmBridge(Node):
    def __init__(self):
        super().__init__('xarm_bridge')

        self.declare_parameter('robot_ip', '192.168.5.240')
        self.declare_parameter(
            'is_radian',
            True,
            ParameterDescriptor(
                description='ROS ufactory 드라이버와 동일하게 기본 라디안.',
            ),
        )
        self.declare_parameter('init_mode', 1)
        self.declare_parameter(
            'init_state',
            0,
            ParameterDescriptor(
                description='set_state 값 (0=동작 가능한 motion state 등, SDK 문서 참고)',
            ),
        )
        self.declare_parameter('motion_enable_servo_id', 8)
        self.declare_parameter('motion_enable', True)

        self.create_service(Trigger, '/arm/init_set', self._on_init_set)
        self.get_logger().info('xArm Bridge 시작: `/arm/init_set` (SDK 직결)')

        self._init_requested = False

        # 7셀 리튬 폴리머 배터리 잔량 연산용 파라미터 (단위: V)
        self.V_MAX = 29.4  # 4.2V * 7
        self.V_MIN = 23.1  # 3.3V * 7

        # 2.5초 주기로 배터리 전압을 읽어오는 타이머 생성
        self.battery_timer = self.create_timer(2.5, self._on_battery_monitor)


    def _resolve_robot_ip(self):
        ip = (
            self.get_parameter('robot_ip')
            .get_parameter_value()
            .string_value
            .strip()
        )
        if ip:
            return ip
        return os.environ.get('XARM_BRIDGE_ROBOT_IP', '').strip()

    # 타이머를 이용해 로봇 제어기 입력 전압 및 퍼센티지 로깅 수행
    def _on_battery_monitor(self):
        robot_ip = self._resolve_robot_ip()
        if not robot_ip:
            return

        is_radian = self.get_parameter('is_radian').get_parameter_value().bool_value

        # 백그라운드 소켓 블로킹을 최소화하기 위해 타임아웃을 짧게 주거나 짧은 커넥션 유지
        arm = None
        try:
            # 상태 모니터링용 인스턴스 생성
            arm = XArmAPI(robot_ip, is_radian=is_radian, do_not_open=True)
            arm.connect()

            if not arm.connected:
                return

            # SDK로부터 실시간 공급 전압(Volt) 획득
            code, volt = arm.get_robot_volt()
            
            if _code_ok(code):
                # 7S 리튬폴리머 기준 선형 퍼센티지(%) 연산
                if volt >= self.V_MAX:
                    percentage = 100.0
                elif volt <= self.V_MIN:
                    percentage = 0.0
                else:
                    percentage = ((volt - self.V_MIN) / (self.V_MAX - self.V_MIN)) * 100.0

                # ANSI 이스케이프 코드: \033[92m (밝은 초록색) / \033[0m (색상 초기화)
                GREEN_COLOR = "\033[92m"
                RESET_COLOR = "\033[0m"

                self.get_logger().info(
                    f'{GREEN_COLOR}[xArm Battery] Voltage: {volt:.2f}V | '
                    f'Remaining: {percentage:.1f}% (7S Li-Po Status){RESET_COLOR}'
                )
        except Exception:
            # 주행이나 초기화 도중 소켓 간섭 등으로 인한 예외 메시지는 스트리밍 방해를 방지하기 위해 묵음 처리
            pass
        finally:
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:
                    pass



    def _run_sdk_init(self):
        robot_ip = self._resolve_robot_ip()
        if not robot_ip:
            self.get_logger().error(
                'robot_ip 가 비었습니다. '
                '`--ros-args -p robot_ip:=<IP>` 또는 환경변수 XARM_BRIDGE_ROBOT_IP 를 설정하세요.'
            )
            return

        is_radian = self.get_parameter('is_radian').get_parameter_value().bool_value

        init_mode = int(self.get_parameter('init_mode').get_parameter_value().integer_value)
        init_state = int(self.get_parameter('init_state').get_parameter_value().integer_value)
        servo_id = int(
            self.get_parameter('motion_enable_servo_id').get_parameter_value().integer_value
        )
        motion_on = self.get_parameter('motion_enable').get_parameter_value().bool_value

        self.get_logger().info(
            f'SDK init_set 시작 robot_ip={robot_ip} '
            f'mode={init_mode} state={init_state} motion_enable={motion_on} servo_id={servo_id}'
        )

        arm = None
        try:
            arm = XArmAPI(robot_ip, is_radian=is_radian)

            if not arm.connected:
                self.get_logger().error('SDK 연결 실패: connected=False (IP/네트워크/다중 연결 확인)')
                return

            steps = (
                ('clean_error', lambda: arm.clean_error()),
                (
                    'motion_enable',
                    lambda: arm.motion_enable(
                        enable=motion_on,
                        servo_id=None if servo_id <= 0 or servo_id == 8 else servo_id,
                    ),
                ),
                ('set_mode', lambda: arm.set_mode(mode=init_mode)),
                ('set_state', lambda: arm.set_state(state=init_state)),
            )

            for label, fn in steps:
                code = fn()
                if _code_ok(code):
                    self.get_logger().info(f'{label}: code={code}')
                else:
                    self.get_logger().error(f'{label}: 실패 code={code}')
                    return

            self.get_logger().info('init_set 시퀀스 완료')
        except Exception as e:
            self.get_logger().error(f'SDK 초기화 예외: {e}')
        finally:
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:
                    pass

    # request, response 시그니처 필수
    def _on_init_set(self, request, response):
        self._init_requested = True
        response.success = True
        response.message = 'init_set 요청 수신 — 메인 루프에서 SDK 시퀀스 실행'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = XArmBridge()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

            if not node._init_requested:
                continue

            node._init_requested = False
            node._run_sdk_init()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
