#!/usr/bin/env python3
"""
Keyboard → MoveIt Servo delta_twist_cmds 컨트롤러

3d_master_controller.py(SpaceMouse) 와 동일한 servo 시퀀스/상태 머신을 쓰고,
입력만 joy 대신 터미널 키보드로 받는다. teleop_twist_keyboard 는 키를 뗀
시점을 알 수 없고 버튼(토글) 개념도 없어서, /dev/tty 를 직접 읽는다.

키 매핑 (command_frame_id 기준, rx/ry 는 사용하지 않음):
  w / s : +x / -x
  a / d : -y / +y
  r / f : +z / -z
  q / e : -rz / +rz
  1     : gripper open/close 토글
  2     : servo ON/OFF 토글 (+ tool_controller ↔ tool_forward_controller 전환)
  space : 즉시 정지 (눌린 키 해제)

[키 hold 판정]
  터미널은 key-release 이벤트가 없고, 키를 누르고 있으면
  "첫 입력 → (OS repeat delay, 보통 ~0.5s) → 반복 입력(~30Hz)" 순서로 들어온다.
  - 첫 입력 후 key_first_timeout_sec 동안은 눌린 것으로 간주 (repeat delay 를 덮음)
  - 반복 입력이 들어오기 시작하면 key_repeat_timeout_sec 안에 다음 입력이 없을 때
    키를 뗀 것으로 판단 → 손을 떼면 빠르게 정지한다.
  터미널 특성상 동시에 여러 키를 누르면 마지막 키만 반복되므로 한 번에 한 축만 움직인다.

[점프 방지 설계] — 3d_master_controller.py 와 동일
  1. stop_servo  → 내부 desired_positions 초기화
  2. 대기        → servo 출력 버퍼 비우기
  3. start_servo → 현재 joint_states 로 재시드
  4. hold 구간   → zero-twist 강제 + joint velocity ≈ 0 확인 후 키 입력 반영
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool as BoolMsg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger

# 3d_master_controller.py 와 동일한 토픽 (arm_move_action_server / text_status_overlay 연동)
_SERVO_OFF_REQUEST_TOPIC = '/servo_mode/request_off'
_STATE_TEXT_TOPIC = '/servo_mode/state_text'
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

_ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
_VEL_ZERO_THRESHOLD = 0.005   # rad/s — 이 이하면 정지로 간주
_STALL_DETECT_SEC   = 0.4     # 명령 non-zero 인데 로봇 정지가 이 시간 이상이면 stall

_STATE_INACTIVE = 'inactive'  # servo 미시작 — 아무것도 publish 안 함
_STATE_HOLD     = 'hold'      # servo 시작됨 — zero-twist 강제 (안정화 대기)
_STATE_ACTIVE   = 'active'    # 안정화 완료 — 키 입력 publish

# key → (축, 방향). 축: lx, ly, lz, rz
_MOTION_KEYS = {
    'w': ('lx', +1.0), 's': ('lx', -1.0),
    'a': ('ly', -1.0), 'd': ('ly', +1.0),
    'r': ('lz', +1.0), 'f': ('lz', -1.0),
    'q': ('rz', -1.0), 'e': ('rz', +1.0),
}
_KEY_GRIPPER_TOGGLE = '1'
_KEY_SERVO_TOGGLE   = '2'
_KEY_STOP           = ' '

_HELP_TEXT = """
──────────── Keyboard Servo Teleop ────────────
   w/s : +x / -x        q/e : -rz / +rz
   a/d : -y / +y        r/f : +z  / -z
   1   : gripper toggle  2   : servo ON/OFF
   space : stop          Ctrl-C : quit
────────────────────────────────────────────────
"""


class KeyboardServoController(Node):
    def __init__(self):
        super().__init__('keyboard_servo_controller')

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter('servo_node_name',      'servo_server')
        self.declare_parameter('command_frame_id',     'tool_tcp_link')
        self.declare_parameter('publish_hz',           50.0)
        # speed_units 기준 (m/s, rad/s)
        self.declare_parameter('linear_speed',         0.05)
        self.declare_parameter('angular_speed',        0.3)
        self.declare_parameter('key_first_timeout_sec',  0.6)
        self.declare_parameter('key_repeat_timeout_sec', 0.15)
        self.declare_parameter('stop_to_start_sec',    0.4)
        self.declare_parameter('hold_after_start_sec', 1.0)
        # gripper
        self.declare_parameter('tool_topic',           '/body/tool_forward_controller/commands')
        self.declare_parameter('gripper_open',         0.05)
        self.declare_parameter('gripper_close',        0.0)
        self.declare_parameter('gripper_init_open',    False)
        # tool controller 전환
        self.declare_parameter('body_cm_ns',              '/body/controller_manager')
        self.declare_parameter('tool_forward_controller', 'tool_forward_controller')
        self.declare_parameter('tool_original_controller','tool_controller')
        # true 면 노드 기동 시 바로 SERVO ON (3d_master 와 동일 동작)
        self.declare_parameter('auto_start_servo',     True)

        servo_ns                 = self.get_parameter('servo_node_name').value
        self._frame_id           = self.get_parameter('command_frame_id').value
        hz                       = float(self.get_parameter('publish_hz').value)
        self._lin_speed          = float(self.get_parameter('linear_speed').value)
        self._ang_speed          = float(self.get_parameter('angular_speed').value)
        self._key_first_timeout  = float(self.get_parameter('key_first_timeout_sec').value)
        self._key_repeat_timeout = float(self.get_parameter('key_repeat_timeout_sec').value)
        self._stop_to_start_sec  = float(self.get_parameter('stop_to_start_sec').value)
        self._hold_min_sec       = float(self.get_parameter('hold_after_start_sec').value)
        self._tool_topic         = self.get_parameter('tool_topic').value
        self._gripper_open_pos   = float(self.get_parameter('gripper_open').value)
        self._gripper_close_pos  = float(self.get_parameter('gripper_close').value)
        gripper_init_open        = bool(self.get_parameter('gripper_init_open').value)
        body_cm_ns               = self.get_parameter('body_cm_ns').value
        self._tool_fwd_ctrl      = self.get_parameter('tool_forward_controller').value
        self._tool_orig_ctrl     = self.get_parameter('tool_original_controller').value
        auto_start               = bool(self.get_parameter('auto_start_servo').value)

        # ── 런타임 상태 ─────────────────────────────────────────────────────
        self._state         = _STATE_INACTIVE
        self._hold_end_time = 0.0
        self._arm_vel_zero  = False
        self._gripper_is_open: bool = gripper_init_open
        self._delayed_start_timer = None
        self._stall_start_t: float | None = None
        self._stalled: bool = False

        # 현재 눌린 motion 키 (터미널 특성상 하나만 유지)
        self._key_lock = threading.Lock()
        self._active_key: str | None = None
        self._key_pressed_t = 0.0   # 최초 입력 시각
        self._key_last_t    = 0.0   # 마지막 입력 시각
        self._key_repeating = False

        self._cbg = ReentrantCallbackGroup()

        # ── 서비스 클라이언트 ────────────────────────────────────────────────
        self._start_srv = self.create_client(
            Trigger, f'/{servo_ns}/start_servo', callback_group=self._cbg,
        )
        self._stop_srv = self.create_client(
            Trigger, f'/{servo_ns}/stop_servo', callback_group=self._cbg,
        )
        self._ctrl_switch_srv = self.create_client(
            SwitchController,
            f'{body_cm_ns}/switch_controller',
            callback_group=self._cbg,
        )

        # ── 퍼블리셔 / 구독 ──────────────────────────────────────────────────
        self._twist_pub = self.create_publisher(
            TwistStamped, f'/{servo_ns}/delta_twist_cmds', 10,
        )
        self._tool_pub = self.create_publisher(
            Float64MultiArray, self._tool_topic, 10,
        )
        self._state_text_pub = self.create_publisher(
            StringMsg, _STATE_TEXT_TOPIC, _LATCHED_QOS,
        )
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10, callback_group=self._cbg,
        )
        self.create_subscription(
            BoolMsg, _SERVO_OFF_REQUEST_TOPIC,
            self._on_request_config_mode, 10, callback_group=self._cbg,
        )

        # ── 타이머 ──────────────────────────────────────────────────────────
        self._publish_timer = self.create_timer(
            1.0 / hz, self._publish_twist, callback_group=self._cbg,
        )
        if auto_start:
            self._startup_timer = self.create_timer(
                0.5, self._startup_begin, callback_group=self._cbg,
            )

        self.get_logger().info(
            f'KeyboardServoController ready. '
            f'servo=/{servo_ns}, frame={self._frame_id}, hz={hz}, '
            f'lin={self._lin_speed}m/s, ang={self._ang_speed}rad/s'
        )
        self._publish_state_text()

    # ═══════════════════════════════════════════════════════════════════════
    # 키 입력 (키보드 스레드에서 호출)
    # ═══════════════════════════════════════════════════════════════════════

    def on_key(self, key: str):
        key = key.lower()
        now = time.monotonic()

        if key in _MOTION_KEYS:
            with self._key_lock:
                if key == self._active_key:
                    self._key_repeating = True
                else:
                    self._active_key    = key
                    self._key_pressed_t = now
                    self._key_repeating = False
                self._key_last_t = now
        elif key == _KEY_STOP:
            self._release_key()
        elif key == _KEY_GRIPPER_TOGGLE:
            self._toggle_gripper()
        elif key == _KEY_SERVO_TOGGLE:
            self._release_key()
            self._toggle_servo_mode()

    def _release_key(self):
        with self._key_lock:
            self._active_key = None
            self._key_repeating = False

    def _current_key(self) -> str | None:
        """timeout 을 반영한 현재 눌린 motion 키."""
        with self._key_lock:
            if self._active_key is None:
                return None
            timeout = (self._key_repeat_timeout if self._key_repeating
                       else self._key_first_timeout)
            if time.monotonic() - self._key_last_t > timeout:
                self._active_key = None
                self._key_repeating = False
                return None
            return self._active_key

    def _toggle_gripper(self):
        self._gripper_is_open = not self._gripper_is_open
        state_str = 'open' if self._gripper_is_open else 'close'
        pos = self._gripper_open_pos if self._gripper_is_open else self._gripper_close_pos
        self.get_logger().info(f'Key 1: gripper toggle → {state_str} ({pos:.4f})')

    # ═══════════════════════════════════════════════════════════════════════
    # servo 상태 문자열 발행 (RViz 표시는 text_status_overlay.py 가 담당)
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_state_text(self):
        if self._state == _STATE_INACTIVE:
            level, text = 'info', 'ARM: PLANNING (config)'
        elif self._state == _STATE_HOLD:
            level, text = 'warn', 'ARM: SERVO (안정화 중...)'
        else:
            level, text = 'error', 'ARM: SERVO ACTIVE (Keyboard)'
        self._state_text_pub.publish(StringMsg(data=f'{level}:{text}'))

    # ═══════════════════════════════════════════════════════════════════════
    # servo 초기화 시퀀스: stop → delay → start
    # ═══════════════════════════════════════════════════════════════════════

    def _startup_begin(self):
        self._startup_timer.cancel()
        self._servo_on('Startup')

    def _servo_on(self, log_label: str):
        """tool_original → tool_forward 전환 + stop → delay → start."""
        self.get_logger().info(
            f'{log_label}: SERVO ON ({self._tool_orig_ctrl} → {self._tool_fwd_ctrl})'
        )
        self._switch_controllers_async(
            activate=[self._tool_fwd_ctrl],
            deactivate=[self._tool_orig_ctrl],
            label=f'{log_label}: activate tool_forward',
        )
        if not self._stop_srv.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('stop_servo 서비스 없음. start_servo 직접 시도.')
            self._do_start_servo()
            return
        fut = self._stop_srv.call_async(Trigger.Request())
        fut.add_done_callback(self._on_pre_stop_done)

    def _on_pre_stop_done(self, fut):
        try:
            res = fut.result()
            self.get_logger().info(f'Pre-start stop_servo: {res.message}')
        except Exception as e:
            self.get_logger().warn(f'Pre-start stop_servo 오류 (무시): {e}')

        self._delayed_start_timer = self.create_timer(
            self._stop_to_start_sec, self._fire_delayed_start, callback_group=self._cbg,
        )

    def _fire_delayed_start(self):
        if self._delayed_start_timer is not None:
            self._delayed_start_timer.cancel()
            self._delayed_start_timer = None
        self._do_start_servo()

    def _do_start_servo(self):
        if not self._start_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('start_servo 서비스를 찾지 못했습니다.')
            return
        self.get_logger().info('start_servo 호출 (joint_states 에서 desired_positions 재시드)...')
        fut = self._start_srv.call_async(Trigger.Request())
        fut.add_done_callback(self._on_start_done)

    def _on_start_done(self, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().error(f'start_servo 오류: {e}')
            return

        if res.success:
            self._hold_end_time = time.monotonic() + self._hold_min_sec
            self._arm_vel_zero  = False
            self._stall_reset()
            self._state         = _STATE_HOLD
            self._publish_state_text()
            self.get_logger().info(
                f'Servo started: {res.message}  '
                f'[HOLD {self._hold_min_sec:.1f}s + arm velocity≈0 대기]'
            )
        else:
            self.get_logger().warn(f'start_servo 실패: {res.message}')

    # ═══════════════════════════════════════════════════════════════════════
    # servo stop / toggle
    # ═══════════════════════════════════════════════════════════════════════

    def _do_stop_servo(self, log_label: str = 'stop_servo'):
        self._state = _STATE_INACTIVE
        self._publish_state_text()
        if not self._stop_srv.service_is_ready():
            self.get_logger().warn(f'{log_label}: stop_servo 서비스 미준비.')
            return
        fut = self._stop_srv.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f'{log_label}: {f.result().message}'
                if not f.exception() else str(f.exception())
            )
        )

    def _toggle_servo_mode(self):
        if self._state == _STATE_INACTIVE:
            # 서비스 대기(wait_for_service)가 키보드 스레드를 막지 않도록 분리
            threading.Thread(
                target=self._servo_on, args=('Key 2',), daemon=True,
            ).start()
        else:
            self._turn_servo_off('Key 2')

    def _turn_servo_off(self, log_label: str):
        self.get_logger().info(
            f'{log_label}: SERVO OFF ({self._tool_fwd_ctrl} → {self._tool_orig_ctrl})'
        )
        self._do_stop_servo(f'{log_label}: SERVO OFF')
        self._switch_controllers_async(
            activate=[self._tool_orig_ctrl],
            deactivate=[self._tool_fwd_ctrl],
            label=f'{log_label}: restore tool_controller',
        )

    def _on_request_config_mode(self, msg: BoolMsg):
        """arm_move_action_server 등이 config 모드 전환을 요청 — 키 2 OFF 와 동일."""
        if not msg.data or self._state == _STATE_INACTIVE:
            return
        self._release_key()
        self._turn_servo_off('External request (e.g. arm_move_action_server)')

    def destroy_node(self):
        self._do_stop_servo('shutdown stop_servo')
        self._switch_controllers_sync(
            activate=[self._tool_orig_ctrl],
            deactivate=[self._tool_fwd_ctrl],
            label='shutdown: restore tool_controller',
            timeout_sec=3.0,
        )
        super().destroy_node()

    # ═══════════════════════════════════════════════════════════════════════
    # controller switch 헬퍼
    # ═══════════════════════════════════════════════════════════════════════

    def _make_switch_request(self, activate: list, deactivate: list) -> SwitchController.Request:
        req = SwitchController.Request()
        req.activate_controllers   = activate
        req.deactivate_controllers = deactivate
        req.strictness   = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        return req

    def _switch_controllers_async(self, activate: list, deactivate: list, label: str) -> None:
        if not self._ctrl_switch_srv.service_is_ready():
            self.get_logger().warn(
                f'{label}: switch_controller 서비스 미준비, 잠시 후 재시도합니다.'
            )
            timer = self.create_timer(1.0, lambda: (
                timer.cancel() or
                self._switch_controllers_async(activate, deactivate, label + ' [retry]')
            ), callback_group=self._cbg)
            return

        fut = self._ctrl_switch_srv.call_async(
            self._make_switch_request(activate, deactivate)
        )
        fut.add_done_callback(lambda f: self._on_switch_done(f, label))

    def _switch_controllers_sync(self, activate: list, deactivate: list,
                                 label: str, timeout_sec: float = 3.0) -> bool:
        if not self._ctrl_switch_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{label}: switch_controller 서비스 없음, 건너뜀.')
            return False

        fut = self._ctrl_switch_srv.call_async(
            self._make_switch_request(activate, deactivate)
        )
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)

        if fut.done():
            return self._on_switch_done(fut, label)
        self.get_logger().warn(f'{label}: 타임아웃 ({timeout_sec}s).')
        return False

    def _on_switch_done(self, fut, label: str) -> bool:
        try:
            res = fut.result()
            if res.ok:
                self.get_logger().info(f'{label}: 성공.')
                return True
            self.get_logger().warn(f'{label}: controller_manager 가 ok=false 반환.')
            return False
        except Exception as e:
            self.get_logger().error(f'{label}: 오류 — {e}')
            return False

    # ═══════════════════════════════════════════════════════════════════════
    # JointState 콜백
    # ═══════════════════════════════════════════════════════════════════════

    def _joint_state_callback(self, msg: JointState):
        vels = []
        for jn in _ARM_JOINT_NAMES:
            if jn not in msg.name:
                return
            idx = msg.name.index(jn)
            if idx >= len(msg.velocity):
                return
            vels.append(abs(msg.velocity[idx]))
        self._arm_vel_zero = (max(vels) < _VEL_ZERO_THRESHOLD)

    # ═══════════════════════════════════════════════════════════════════════
    # 주기 publish (상태 머신)
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_twist(self):
        if self._state == _STATE_INACTIVE:
            return

        if self._state == _STATE_HOLD:
            if time.monotonic() >= self._hold_end_time and self._arm_vel_zero:
                self._state = _STATE_ACTIVE
                self._publish_state_text()
                self.get_logger().info('Hold released → ACTIVE. Keyboard input enabled.')
            else:
                self._publish_zero_twist()
                self._publish_gripper_cmd()
                return

        key = self._current_key()
        cmd = {'lx': 0.0, 'ly': 0.0, 'lz': 0.0, 'rz': 0.0}
        if key is not None:
            axis, sign = _MOTION_KEYS[key]
            speed = self._ang_speed if axis == 'rz' else self._lin_speed
            cmd[axis] = sign * speed

        # ── Stall 감지/해제 ────────────────────────────────────────────
        commanding = key is not None
        now = time.monotonic()

        if self._stalled:
            if not commanding:
                self._stalled = False
                self._stall_start_t = None
                self.get_logger().info('Stall cleared: key released → motion re-enabled.')
            else:
                self.get_logger().warn(
                    'STALLED: 키를 떼면 재개됩니다.', throttle_duration_sec=1.0,
                )
                self._publish_zero_twist()
                self._publish_gripper_cmd()
                return
        elif commanding and self._arm_vel_zero:
            if self._stall_start_t is None:
                self._stall_start_t = now
            elif now - self._stall_start_t >= _STALL_DETECT_SEC:
                self._stalled = True
                self._stall_start_t = None
                self.get_logger().warn(
                    f'Motion stall detected ({_STALL_DETECT_SEC}s): '
                    '키 명령 차단. 키를 떼면 해제됩니다.'
                )
                self._publish_zero_twist()
                self._publish_gripper_cmd()
                return
        else:
            self._stall_start_t = None

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x  = cmd['lx']
        msg.twist.linear.y  = cmd['ly']
        msg.twist.linear.z  = cmd['lz']
        msg.twist.angular.z = cmd['rz']
        self._twist_pub.publish(msg)
        self._publish_gripper_cmd()

    def _stall_reset(self):
        self._stalled       = False
        self._stall_start_t = None

    def _publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._twist_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # gripper
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_gripper_cmd(self):
        pos = self._gripper_open_pos if self._gripper_is_open else self._gripper_close_pos
        cmd = Float64MultiArray()
        cmd.data = [pos]
        self._tool_pub.publish(cmd)


# ═══════════════════════════════════════════════════════════════════════════
# 터미널 키 입력 스레드
# ═══════════════════════════════════════════════════════════════════════════

def _open_tty():
    """ros2 launch 로 실행되면 stdin 이 터미널이 아니므로 /dev/tty 를 직접 연다."""
    if sys.stdin.isatty():
        return sys.stdin.fileno(), False
    try:
        return os.open('/dev/tty', os.O_RDONLY), True
    except OSError:
        return None, False


def _keyboard_loop(node: KeyboardServoController, stop_event: threading.Event):
    fd, opened = _open_tty()
    if fd is None:
        node.get_logger().error(
            '키보드 입력용 터미널(/dev/tty)을 열 수 없습니다. '
            '터미널에서 ros2 run / ros2 launch 로 실행하세요.'
        )
        return

    old_attrs = termios.tcgetattr(fd)
    try:
        # cbreak: 한 글자씩 즉시 읽되 Ctrl-C(SIGINT) 는 그대로 동작
        tty.setcbreak(fd)
        # echo 끄기
        attrs = termios.tcgetattr(fd)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)

        print(_HELP_TEXT, flush=True)
        while not stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.05)
            if not ready:
                continue
            data = os.read(fd, 32)
            if not data:
                break
            for ch in data.decode(errors='ignore'):
                node.on_key(ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        if opened:
            os.close(fd)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()
    kb_thread = threading.Thread(
        target=_keyboard_loop, args=(node, stop_event), daemon=True,
    )
    kb_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        kb_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
