import io
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from urdf_parser_py.urdf import URDF


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# URDF에서 읽어올 joint 이름 목록
_JOINTS_TO_PARSE = ['lift_joint', 'head_joint1', 'head_joint2']


class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        # ── 파라미터 ─────────────────────────────────────────────────────────
        self.declare_parameter('lift_step',     0.02)  # 타이머 1회당 lift 이동량 (m)
        self.declare_parameter('head_step',     0.02)   # 타이머 1회당 head 이동량 (rad)
        self.declare_parameter('control_hz',    50.0)   # 제어 명령 주기 (Hz)
        self.declare_parameter('joint_states_topic', '/joint_states')

        self._lift_step    = self.get_parameter('lift_step').value
        self._head_step    = self.get_parameter('head_step').value
        self._control_hz   = self.get_parameter('control_hz').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # ── 상태 ─────────────────────────────────────────────────────────────
        self.prev_buttons  = []
        self._current_axes = []           # Joy axes 전체 저장
        self._current_mode = 'normal'

        # lift
        self._current_lift_pos = None
        self._lift_direction   = 0            # +1: 상승, -1: 하강, 0: 정지
        self._lift_min         = -math.inf
        self._lift_max         =  math.inf

        # head (axes 로 직접 방향 결정)
        self._current_head1_pos = None
        self._current_head2_pos = None
        self._head1_min         = -math.inf
        self._head1_max         =  math.inf
        self._head2_min         = -math.inf
        self._head2_max         =  math.inf

        # URDF 파싱 완료 플래그 (모든 joint limit 수신 후 콜백 무시)
        self._limits_loaded = set()  # 파싱된 joint 이름 집합

        self._cbg = ReentrantCallbackGroup()

        # ── 구독 ─────────────────────────────────────────────────────────────
        self.create_subscription(
            Joy, 'joy', self.joy_callback, 10, callback_group=self._cbg,
        )
        self.create_subscription(
            JointState, joint_states_topic,
            self._joint_state_callback, 10, callback_group=self._cbg,
        )
        self.create_subscription(
            String, '/robot_description',
            self._on_robot_description, _LATCHED_QOS, callback_group=self._cbg,
        )
        self.create_subscription(
            String, '/controller_mode_switcher/mode',
            self._mode_callback, _LATCHED_QOS, callback_group=self._cbg,
        )

        # ── 퍼블리셔 ─────────────────────────────────────────────────────────
        self._mode_switch_pub = self.create_publisher(
            Bool, '/controller_mode_switcher/switch_mode_cmd', 10,
        )
        self._lift_pub = self.create_publisher(
            Float64MultiArray, '/body/lift_forward_controller/commands', 10,
        )
        self._head_pub = self.create_publisher(
            Float64MultiArray, '/body/head_forward_controller/commands', 10,
        )

        # ── 제어 타이머 ───────────────────────────────────────────────────────
        self.create_timer(
            1.0 / self._control_hz,
            self._control_timer_callback,
            callback_group=self._cbg,
        )

        self.get_logger().info(
            f'Master controller initialized. '
            f'lift_step={self._lift_step} m, head_step={self._head_step} rad '
            f'@ {self._control_hz} Hz  (joint limits: pending /robot_description)'
        )

    # ── 구독 콜백 ────────────────────────────────────────────────────────────

    def _on_robot_description(self, msg: String):
        """
        /robot_description(TRANSIENT_LOCAL) 에서 URDF를 파싱해
        lift_joint / head_joint1 / head_joint2 의 position limit을 설정한다.
        모든 joint limit이 로드되면 이후 메시지는 무시한다.
        """
        if self._limits_loaded >= set(_JOINTS_TO_PARSE):
            return  # 이미 모두 로드됨 — 중복 처리 방지

        try:
            _stderr, sys.stderr = sys.stderr, io.StringIO()
            robot = URDF.from_xml_string(msg.data)
            sys.stderr = _stderr

            joint_map = {j.name: j for j in robot.joints}
            for name in _JOINTS_TO_PARSE:
                if name in self._limits_loaded:
                    continue  # 이미 이 joint는 처리됨
                j = joint_map.get(name)
                if j and j.limit:
                    lo, hi = j.limit.lower, j.limit.upper
                    if name == 'lift_joint':
                        self._lift_min, self._lift_max = lo, hi
                    elif name == 'head_joint1':
                        self._head1_min, self._head1_max = lo, hi
                    elif name == 'head_joint2':
                        self._head2_min, self._head2_max = lo, hi
                    self._limits_loaded.add(name)
                    self.get_logger().info(
                        f'{name} limit loaded: [{lo:.3f}, {hi:.3f}]'
                    )

            if self._limits_loaded >= set(_JOINTS_TO_PARSE):
                self.get_logger().info('All joint limits loaded. Ignoring further /robot_description messages.')

        except Exception as e:
            self.get_logger().warn(f'URDF parse failed — clamp disabled: {e}')

    def _joint_state_callback(self, msg: JointState):
        names = msg.name
        if 'lift_joint' in names:
            self._current_lift_pos = msg.position[names.index('lift_joint')]
        if 'head_joint1' in names:
            self._current_head1_pos = msg.position[names.index('head_joint1')]
        if 'head_joint2' in names:
            self._current_head2_pos = msg.position[names.index('head_joint2')]

    def _mode_callback(self, msg: String):
        self._current_mode = msg.data

    # ── 제어 타이머 콜백 ─────────────────────────────────────────────────────

    def _control_timer_callback(self):
        """주기적으로 lift / head 명령을 발행한다."""
        if self._current_mode != 'forward':
            return

        self._tick_lift()
        self._tick_head()

    def _tick_lift(self):
        if self._lift_direction == 0 or self._current_lift_pos is None:
            return

        target = self._current_lift_pos + self._lift_direction * self._lift_step
        target = max(self._lift_min, min(self._lift_max, target))

        if target == self._current_lift_pos:
            return

        cmd = Float64MultiArray()
        cmd.data = [target]
        self._lift_pub.publish(cmd)

    def _tick_head(self):
        if self._current_head1_pos is None or self._current_head2_pos is None:
            return
        if not self._current_axes:
            return

        # axes[4] → head_joint1 방향,  axes[5] → head_joint2 방향
        dir1 = self._axis_to_dir(self._current_axes, 4)
        dir2 = self._axis_to_dir(self._current_axes, 5)

        if dir1 == 0 and dir2 == 0:
            return

        t1 = self._current_head1_pos + dir1 * self._head_step
        t2 = self._current_head2_pos + dir2 * self._head_step
        t1 = max(self._head1_min, min(self._head1_max, t1))
        t2 = max(self._head2_min, min(self._head2_max, t2))

        cmd = Float64MultiArray()
        cmd.data = [t1, t2]
        self._head_pub.publish(cmd)

    @staticmethod
    def _axis_to_dir(axes: list, idx: int) -> int:
        """디지털 축 값(±1, 0)을 정수 방향으로 변환한다."""
        if idx >= len(axes):
            return 0
        val = axes[idx]
        if val > 0.5:
            return +1
        if val < -0.5:
            return -1
        return 0

    # ── 모드 전환 토픽 발행 ──────────────────────────────────────────────────

    def _call_switch_mode(self, data: bool):
        msg = Bool()
        msg.data = data
        self._mode_switch_pub.publish(msg)
        self.get_logger().info(
            f'Mode switch command published: {"forward" if data else "normal"}'
        )

    # ── 조이스틱 콜백 ────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        # axes 전체를 매 메시지마다 갱신 (head 제어에 사용)
        self._current_axes = list(msg.axes)

        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            return

        for i in range(len(msg.buttons)):
            current_val = msg.buttons[i]
            prev_val    = self.prev_buttons[i]

            if current_val != prev_val:
                if current_val == 1:
                    self.on_button_pressed(i)
                else:
                    self.on_button_released(i)

        self.prev_buttons = list(msg.buttons)

    def on_button_pressed(self, index):
        self.get_logger().info(f'Button {index} Pressed!')

        if index == 0:   # Button 1 누름 → FORWARD 모드 (데드맨 스위치 ON)
            self._call_switch_mode(data=True)
        elif index == 5: # Button 6 누름 → 리프트 상승 시작
            self._lift_direction = +1
        elif index == 3: # Button 4 누름 → 리프트 하강 시작
            self._lift_direction = -1

    def on_button_released(self, index):
        self.get_logger().info(f'Button {index} Released!')

        if index == 0:   # Button 1 뗌 → NORMAL 모드 (데드맨 스위치 OFF)
            self._call_switch_mode(data=False)
        elif index == 5: # Button 6 뗌 → 리프트 상승 정지
            self._lift_direction = 0
        elif index == 3: # Button 4 뗌 → 리프트 하강 정지
            self._lift_direction = 0


def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
