#!/usr/bin/env python3
"""
3D Connexion SpaceMouse → MoveIt Servo delta_twist_cmds 컨트롤러

[점프 방지 설계]
  노드 재시작 시 servo 내부 desired_positions 가 이전 세션의 마지막 위치를 기억하는 문제 방지.

  핵심: start_servo 만 호출하면 servo가 STARTED 상태인 경우 재초기화가 일어나지 않는다.
  올바른 순서:
    1. stop_servo  → 내부 desired_positions 초기화 (STOPPED 상태로 전환)
    2. 0.3 s 대기  → servo 출력 버퍼 비우기
    3. start_servo → 현재 joint_states 로 desired_positions 재시드
    4. hold 구간   → zero-twist 강제 전송 (Butterworth 필터 과도 응답 억제)
    5. 추가 조건   → joint velocity ≈ 0 확인 후 SpaceMouse 입력 반영

  노드 종료 시에도 stop_servo 를 호출하여 servo 상태를 STOPPED 로 남긴다.

버튼 동작:
  버튼 0: gripper open
  버튼 1: gripper close

SpaceMouse joy 축 기본 매핑 (joy_node 드라이버 기준):
  axes[0]: tx  (좌/우)   axes[3]: rx (roll)
  axes[1]: ty  (전/후)   axes[4]: ry (pitch)
  axes[2]: tz  (상/하)   axes[5]: rz (yaw)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

_ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
_VEL_ZERO_THRESHOLD  = 0.005   # rad/s — 이 이하면 정지로 간주
# stall 감지: 명령은 non-zero인데 로봇이 이 시간(초) 이상 정지해 있으면 stall로 판단
_STALL_DETECT_SEC    = 0.4
# stall 해제: SpaceMouse 모든 축이 deadband 이하로 돌아와야 재개
# (사용자가 의도적으로 조이스틱을 놓아야 한다)

# _state 값
_STATE_INACTIVE = 'inactive'  # servo 미시작 — 아무것도 publish 안 함
_STATE_HOLD     = 'hold'      # servo 시작됨 — zero-twist 강제 (안정화 대기)
_STATE_ACTIVE   = 'active'    # 안정화 완료 — SpaceMouse 입력 그대로 publish
# HOLD → ACTIVE 는 (시간 경과 AND velocity≈0) 조건이 동시 충족될 때만 전이.
# ACTIVE → HOLD 전이는 없음 (움직임 중 velocity > 0 으로 재진입하는 버그 방지).


class SpaceMouseServoController(Node):
    def __init__(self):
        super().__init__('spacemouse_servo_controller')

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter('servo_node_name',      'servo_server')
        self.declare_parameter('command_frame_id',     'tool_tcp_link')
        self.declare_parameter('publish_hz',           50.0)
        self.declare_parameter('deadband',             0.05)
        self.declare_parameter('linear_scale',         1.0)
        self.declare_parameter('angular_scale',        1.0)
        # stop → start 사이 대기 시간 (servo 출력 버퍼 비우기)
        self.declare_parameter('stop_to_start_sec',    0.4)
        # start 후 강제 hold 최소 시간 (Butterworth 과도 응답 억제)
        self.declare_parameter('hold_after_start_sec', 1.0)
        # gripper
        self.declare_parameter('tool_topic',           '/body/tool_forward_controller/commands')
        self.declare_parameter('gripper_open',         0.05)
        self.declare_parameter('gripper_close',        0.0)
        self.declare_parameter('btn_gripper_toggle',   0)  # gripper open/close 토글
        self.declare_parameter('gripper_init_open',    True)
        # servo on/off + controller 전환 토글 버튼
        self.declare_parameter('btn_servo_toggle',     1)
        # tool controller 전환 (노드 시작/종료 시 자동 switch)
        self.declare_parameter('body_cm_ns',              '/body/controller_manager')
        self.declare_parameter('tool_forward_controller', 'tool_forward_controller')
        self.declare_parameter('tool_original_controller','tool_controller')
        # 6DOF 축 인덱스
        self.declare_parameter('axis_tx', 0)
        self.declare_parameter('axis_ty', 1)
        self.declare_parameter('axis_tz', 2)
        self.declare_parameter('axis_rx', 3)
        self.declare_parameter('axis_ry', 4)
        self.declare_parameter('axis_rz', 5)
        # 축 방향 반전 (+1 or -1)
        self.declare_parameter('sign_tx',  1)
        self.declare_parameter('sign_ty',  1)
        self.declare_parameter('sign_tz',  1)
        self.declare_parameter('sign_rx',  1)
        self.declare_parameter('sign_ry',  1)
        self.declare_parameter('sign_rz',  1)

        servo_ns                = self.get_parameter('servo_node_name').value
        self._frame_id          = self.get_parameter('command_frame_id').value
        hz                      = float(self.get_parameter('publish_hz').value)
        self._deadband          = float(self.get_parameter('deadband').value)
        self._lin_sc            = float(self.get_parameter('linear_scale').value)
        self._ang_sc            = float(self.get_parameter('angular_scale').value)
        self._stop_to_start_sec = float(self.get_parameter('stop_to_start_sec').value)
        self._hold_min_sec      = float(self.get_parameter('hold_after_start_sec').value)
        self._tool_topic         = self.get_parameter('tool_topic').value
        self._gripper_open_pos   = float(self.get_parameter('gripper_open').value)
        self._gripper_close_pos  = float(self.get_parameter('gripper_close').value)
        self._btn_gripper_toggle = int(self.get_parameter('btn_gripper_toggle').value)
        gripper_init_open        = bool(self.get_parameter('gripper_init_open').value)
        self._btn_servo_toggle   = int(self.get_parameter('btn_servo_toggle').value)
        body_cm_ns              = self.get_parameter('body_cm_ns').value
        self._tool_fwd_ctrl     = self.get_parameter('tool_forward_controller').value
        self._tool_orig_ctrl    = self.get_parameter('tool_original_controller').value

        self._ax_idx = {k: int(self.get_parameter(f'axis_{k}').value)
                        for k in ('tx', 'ty', 'tz', 'rx', 'ry', 'rz')}
        self._ax_sign = {k: float(self.get_parameter(f'sign_{k}').value)
                         for k in ('tx', 'ty', 'tz', 'rx', 'ry', 'rz')}

        # ── 런타임 상태 ─────────────────────────────────────────────────────
        self._latest_axes: list[float] = []
        self._prev_buttons: list[int] = []
        # 명시적 상태 머신: INACTIVE → HOLD → ACTIVE (단방향)
        self._state         = _STATE_INACTIVE
        self._hold_end_time = 0.0   # HOLD 종료 기준 시각 (monotonic)
        self._arm_vel_zero  = False  # joint_states 로 갱신; HOLD 탈출 조건에만 사용
        # gripper 토글 상태 (True = open)
        self._gripper_is_open: bool = gripper_init_open
        # 지연 start 타이머 핸들 (중복 실행 방지)
        self._delayed_start_timer = None
        # stall 감지 상태
        # — 명령 non-zero & 로봇 정지가 _STALL_DETECT_SEC 이상 지속되면 stall 진입
        # — SpaceMouse 가 neutral(deadband 이하)로 돌아와야 stall 해제
        self._stall_start_t: float | None = None
        self._stalled: bool = False

        self._cbg = ReentrantCallbackGroup()

        # ── 서비스 클라이언트 ────────────────────────────────────────────────
        self._start_srv = self.create_client(
            Trigger, f'/{servo_ns}/start_servo', callback_group=self._cbg,
        )
        self._stop_srv = self.create_client(
            Trigger, f'/{servo_ns}/stop_servo', callback_group=self._cbg,
        )
        # /body/controller_manager/switch_controller
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
        self.create_subscription(
            Joy, 'joy', self._joy_callback, 10, callback_group=self._cbg,
        )
        # joint velocity 감시: hold 종료 조건
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10, callback_group=self._cbg,
        )

        # ── 타이머 ──────────────────────────────────────────────────────────
        self._publish_timer = self.create_timer(
            1.0 / hz, self._publish_twist, callback_group=self._cbg,
        )
        # 노드 기동 후 stop → start 시퀀스 시작
        self._startup_timer = self.create_timer(
            0.5, self._startup_begin, callback_group=self._cbg,
        )

        self.get_logger().info(
            f'SpaceMouseServoController ready. '
            f'servo=/{servo_ns}, frame={self._frame_id}, hz={hz}, '
            f'stop_to_start={self._stop_to_start_sec}s, '
            f'hold_min={self._hold_min_sec}s'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # servo 초기화 시퀀스: stop → delay → start
    # ═══════════════════════════════════════════════════════════════════════

    def _startup_begin(self):
        """노드 기동 후 최초 1회.
        (A) tool_controller → tool_forward_controller 전환
        (B) stop_servo → delay → start_servo
        두 작업은 독립적으로 병행 실행된다.
        """
        self._startup_timer.cancel()

        # (A) 컨트롤러 전환: tool_original → tool_forward
        self._switch_controllers_async(
            activate=[self._tool_fwd_ctrl],
            deactivate=[self._tool_orig_ctrl],
            label='startup: activate tool_forward_controller',
        )

        # (B) servo 초기화
        if not self._stop_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('stop_servo 서비스 없음. start_servo 직접 시도.')
            self._do_start_servo()
            return

        self.get_logger().info('Startup: stop_servo 호출 (이전 세션 상태 초기화)...')
        fut = self._stop_srv.call_async(Trigger.Request())
        fut.add_done_callback(self._on_pre_stop_done)

    def _on_pre_stop_done(self, fut):
        """stop_servo 완료 → delay 후 start_servo."""
        try:
            res = fut.result()
            self.get_logger().info(f'Pre-start stop_servo: {res.message}')
        except Exception as e:
            self.get_logger().warn(f'Pre-start stop_servo 오류 (무시): {e}')

        # stop 후 servo 출력 버퍼가 비워질 시간을 준다.
        # (trajectory controller 가 halt 명령을 소화할 때까지)
        self._delayed_start_timer = self.create_timer(
            self._stop_to_start_sec, self._fire_delayed_start, callback_group=self._cbg,
        )

    def _fire_delayed_start(self):
        """One-shot: delay 만료 후 start_servo 호출."""
        if self._delayed_start_timer is not None:
            self._delayed_start_timer.cancel()
            self._delayed_start_timer = None
        self._do_start_servo()

    def _do_start_servo(self):
        """start_servo 서비스 호출."""
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
            # INACTIVE / ACTIVE → HOLD: 안정화 대기 시작
            self._hold_end_time = time.monotonic() + self._hold_min_sec
            self._arm_vel_zero  = False   # HOLD 탈출 조건 초기화
            self._stall_reset()           # stall 상태 초기화
            self._state         = _STATE_HOLD
            self.get_logger().info(
                f'Servo started: {res.message}  '
                f'[HOLD {self._hold_min_sec:.1f}s + arm velocity≈0 대기]'
            )
        else:
            self.get_logger().warn(f'start_servo 실패: {res.message}')

    # ═══════════════════════════════════════════════════════════════════════
    # servo stop (버튼 또는 종료 시)
    # ═══════════════════════════════════════════════════════════════════════

    def _do_stop_servo(self, log_label: str = 'stop_servo'):
        self._state = _STATE_INACTIVE
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

    def destroy_node(self):
        """종료 시:
        1. stop_servo (servo 상태 초기화)
        2. tool_forward_controller → tool_original_controller 복원 (동기 대기)
        """
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
        # BEST_EFFORT: 이미 목표 상태인 컨트롤러가 있어도 오류로 처리하지 않음
        req.strictness   = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        return req

    def _switch_controllers_async(self,
                                   activate: list,
                                   deactivate: list,
                                   label: str) -> None:
        """비동기 switch_controller 호출 (노드 시작 시 사용)."""
        if not self._ctrl_switch_srv.service_is_ready():
            self.get_logger().warn(
                f'{label}: switch_controller 서비스 미준비, '
                '잠시 후 재시도합니다.',
            )
            # 1초 후 재시도
            timer = self.create_timer(1.0, lambda: (
                timer.cancel() or
                self._switch_controllers_async(activate, deactivate, label + ' [retry]')
            ), callback_group=self._cbg)
            return

        fut = self._ctrl_switch_srv.call_async(
            self._make_switch_request(activate, deactivate)
        )
        fut.add_done_callback(
            lambda f: self._on_switch_done(f, label)
        )

    def _switch_controllers_sync(self,
                                  activate: list,
                                  deactivate: list,
                                  label: str,
                                  timeout_sec: float = 3.0) -> bool:
        """동기 switch_controller 호출 (노드 종료 시 사용).
        spin_until_future_complete 로 블로킹 대기하므로 destroy_node 내에서 안전하다.
        """
        if not self._ctrl_switch_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{label}: switch_controller 서비스 없음, 건너뜀.')
            return False

        fut = self._ctrl_switch_srv.call_async(
            self._make_switch_request(activate, deactivate)
        )
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)

        if fut.done():
            return self._on_switch_done(fut, label)
        else:
            self.get_logger().warn(f'{label}: 타임아웃 ({timeout_sec}s).')
            return False

    def _on_switch_done(self, fut, label: str) -> bool:
        try:
            res = fut.result()
            if res.ok:
                self.get_logger().info(f'{label}: 성공.')
                return True
            else:
                self.get_logger().warn(f'{label}: controller_manager 가 ok=false 반환.')
                return False
        except Exception as e:
            self.get_logger().error(f'{label}: 오류 — {e}')
            return False

    # ═══════════════════════════════════════════════════════════════════════
    # Joy / JointState 콜백
    # ═══════════════════════════════════════════════════════════════════════

    def _joint_state_callback(self, msg: JointState):
        """arm joint velocity 를 감시하여 정지 여부를 판단한다."""
        vels = []
        for jn in _ARM_JOINT_NAMES:
            if jn not in msg.name:
                return
            idx = msg.name.index(jn)
            if idx >= len(msg.velocity):
                return
            vels.append(abs(msg.velocity[idx]))
        self._arm_vel_zero = (max(vels) < _VEL_ZERO_THRESHOLD)

    def _joy_callback(self, msg: Joy):
        self._latest_axes = list(msg.axes)

        buttons = list(msg.buttons)
        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        for i, (prev, cur) in enumerate(zip(self._prev_buttons, buttons)):
            if cur == 1 and prev == 0:
                self._on_button_pressed(i)

        self._prev_buttons = buttons

    def _on_button_pressed(self, index: int):
        if index == self._btn_gripper_toggle:
            self._gripper_is_open = not self._gripper_is_open
            state_str = 'open' if self._gripper_is_open else 'close'
            pos = self._gripper_open_pos if self._gripper_is_open else self._gripper_close_pos
            self.get_logger().info(
                f'Button {index}: gripper toggle → {state_str} ({pos:.4f})'
            )

        elif index == self._btn_servo_toggle:
            self._toggle_servo_mode()

    def _toggle_servo_mode(self):
        """
        INACTIVE  → servo start + tool_original → tool_forward 전환
        HOLD/ACTIVE → servo stop  + tool_forward → tool_original 복원
        """
        if self._state == _STATE_INACTIVE:
            # ── SERVO ON ────────────────────────────────────────────────
            self.get_logger().info(
                'Button servo_toggle: SERVO ON '
                f'({self._tool_orig_ctrl} → {self._tool_fwd_ctrl})'
            )
            self._switch_controllers_async(
                activate=[self._tool_fwd_ctrl],
                deactivate=[self._tool_orig_ctrl],
                label='servo_toggle ON: activate tool_forward',
            )
            # stop → delay → start 시퀀스 (startup_begin 재사용)
            self._do_stop_then_start()
        else:
            # ── SERVO OFF ───────────────────────────────────────────────
            self.get_logger().info(
                'Button servo_toggle: SERVO OFF '
                f'({self._tool_fwd_ctrl} → {self._tool_orig_ctrl})'
            )
            self._do_stop_servo('servo_toggle OFF')
            self._switch_controllers_async(
                activate=[self._tool_orig_ctrl],
                deactivate=[self._tool_fwd_ctrl],
                label='servo_toggle OFF: restore tool_controller',
            )

    def _do_stop_then_start(self):
        """stop_servo → delay → start_servo 시퀀스 (버튼 토글용)."""
        if not self._stop_srv.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('stop_servo 서비스 없음. start_servo 직접 시도.')
            self._do_start_servo()
            return
        fut = self._stop_srv.call_async(Trigger.Request())
        fut.add_done_callback(self._on_pre_stop_done)

    # ═══════════════════════════════════════════════════════════════════════
    # 주기 publish  (상태 머신)
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_twist(self):
        # ── INACTIVE: 아무것도 하지 않음 ─────────────────────────────────
        if self._state == _STATE_INACTIVE:
            return

        # ── HOLD: zero-twist 강제. 탈출 조건 충족 시 ACTIVE 전이 ─────────
        if self._state == _STATE_HOLD:
            time_ok = time.monotonic() >= self._hold_end_time
            vel_ok  = self._arm_vel_zero
            if time_ok and vel_ok:
                self._state = _STATE_ACTIVE
                self.get_logger().info('Hold released → ACTIVE. SpaceMouse input enabled.')
            else:
                self._publish_zero_twist()
                self._publish_gripper_cmd()   # HOLD 중에도 gripper 상태 유지
                self.get_logger().debug(
                    f'[HOLD] time_ok={time_ok} vel_ok={vel_ok} '
                    f'remaining={max(0.0, self._hold_end_time - time.monotonic()):.2f}s',
                    throttle_duration_sec=0.5,
                )
                return

        # ── ACTIVE: SpaceMouse 입력 publish + gripper 상태 동시 publish ──
        # (velocity 체크 없음 — 움직임 중 재진입 차단)
        if not self._latest_axes:
            self._stall_reset()
            self._publish_zero_twist()
            self._publish_gripper_cmd()
            return

        def _get(key: str) -> float:
            idx  = self._ax_idx[key]
            sign = self._ax_sign[key]
            if idx >= len(self._latest_axes):
                return 0.0
            raw = self._latest_axes[idx]
            return 0.0 if abs(raw) < self._deadband else sign * raw

        lx = _get('tx') * self._lin_sc
        ly = _get('ty') * self._lin_sc
        lz = _get('tz') * self._lin_sc
        rx = _get('rx') * self._ang_sc
        ry = _get('ry') * self._ang_sc
        rz = _get('rz') * self._ang_sc

        # ── Stall 감지/해제 ────────────────────────────────────────────
        commanding = (lx != 0.0 or ly != 0.0 or lz != 0.0 or
                      rx != 0.0 or ry != 0.0 or rz != 0.0)
        now = time.monotonic()

        if self._stalled:
            # stall 해제 조건: SpaceMouse 가 neutral 로 돌아온 경우
            if not commanding:
                self._stalled = False
                self._stall_start_t = None
                self.get_logger().info(
                    'Stall cleared: SpaceMouse neutral → motion re-enabled.'
                )
            else:
                self.get_logger().warn(
                    'STALLED: SpaceMouse neutral 로 돌려놓으면 재개됩니다.',
                    throttle_duration_sec=1.0,
                )
                self._publish_zero_twist()
                self._publish_gripper_cmd()
                return
        else:
            if commanding and self._arm_vel_zero:
                # 명령은 있는데 로봇이 정지 → stall 카운트 시작/지속
                if self._stall_start_t is None:
                    self._stall_start_t = now
                elif now - self._stall_start_t >= _STALL_DETECT_SEC:
                    self._stalled = True
                    self._stall_start_t = None
                    self.get_logger().warn(
                        f'Motion stall detected ({_STALL_DETECT_SEC}s): '
                        'SpaceMouse 명령 차단. 조이스틱을 놓아 해제하세요.'
                    )
                    self._publish_zero_twist()
                    self._publish_gripper_cmd()
                    return
            else:
                # 정상 이동 중이거나 SpaceMouse neutral → stall 카운터 초기화
                self._stall_start_t = None

        # ── 정상 publish ───────────────────────────────────────────────
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x  = lx
        msg.twist.linear.y  = ly
        msg.twist.linear.z  = lz
        msg.twist.angular.x = rx
        msg.twist.angular.y = ry
        msg.twist.angular.z = rz
        self._twist_pub.publish(msg)
        self._publish_gripper_cmd()

    def _stall_reset(self):
        """stall 상태 및 카운터를 초기화한다 (HOLD 진입·종료, axes 없음 등)."""
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
        """현재 토글 상태(open/close)를 tool_forward_controller 에 publish.
        _publish_twist 와 동일한 타이머에서 호출되어 같은 주기로 전송된다.
        """
        pos = self._gripper_open_pos if self._gripper_is_open else self._gripper_close_pos
        cmd = Float64MultiArray()
        cmd.data = [pos]
        self._tool_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseServoController()
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
