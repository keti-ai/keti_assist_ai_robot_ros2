#!/usr/bin/env python3
"""
controller_mode_switcher.py
────────────────────────────────────────────────────────────────────────
arm / lift / head / tool 4 개 컨트롤러를 전환하는 노드.

┌──────────────────────────────────────────────────────────────────────┐
│  NORMAL 모드 (기본값, MoveIt / Action 기반)                           │
│    /arm  : xarm7_traj_controller   (JointTrajectoryController)        │
│    /body : lift_controller         (JointTrajectoryController)        │
│            head_controller         (JointTrajectoryController)        │
│            tool_controller         (GripperActionController)          │
├──────────────────────────────────────────────────────────────────────┤
│  FORWARD 모드 (토픽 기반 실시간 제어)                                  │
│    /arm  : xarm7_forward_controller  (ForwardCommandController)       │
│              → /arm/xarm7_forward_controller/commands                 │
│                  std_msgs/Float64MultiArray  [j1..j7]                 │
│    /body : lift_forward_controller   (ForwardCommandController)       │
│              → /body/lift_forward_controller/commands                 │
│                  std_msgs/Float64MultiArray  [lift_joint]             │
│            head_forward_controller   (ForwardCommandController)       │
│              → /body/head_forward_controller/commands                 │
│                  std_msgs/Float64MultiArray  [head_joint1, head_joint2]│
│            tool_forward_controller   (ForwardCommandController)       │
│              → /body/tool_forward_controller/commands                 │
│                  std_msgs/Float64MultiArray  [virtual_gripper_joint]  │
└──────────────────────────────────────────────────────────────────────┘

▸ 토픽 (권장)
    ~/switch_mode_cmd  (std_msgs/Bool)
        data = true  → FORWARD 모드
        data = false → NORMAL  모드 (기본값)

▸ 서비스 (하위 호환 유지)
    ~/switch_mode  (std_srvs/SetBool)
        data = true  → FORWARD 모드
        data = false → NORMAL  모드 (기본값)

▸ 퍼블리셔
    ~/mode  (std_msgs/String, transient_local)
        "normal" 또는 "forward" 를 항상 발행한다.

▸ 파라미터
    arm_cm_ns   (default: /arm/controller_manager)
    body_cm_ns  (default: /body/controller_manager)

▸ 사용 예
    # FORWARD 모드로 전환 (토픽)
    ros2 topic pub --once /controller_mode_switcher/switch_mode_cmd std_msgs/msg/Bool '{data: true}'

    # NORMAL 모드로 복귀 (토픽)
    ros2 topic pub --once /controller_mode_switcher/switch_mode_cmd std_msgs/msg/Bool '{data: false}'

    # FORWARD 모드로 전환 (서비스)
    ros2 service call /controller_mode_switcher/switch_mode std_srvs/srv/SetBool '{data: true}'

    # 현재 모드 확인
    ros2 topic echo /controller_mode_switcher/mode
"""

import threading
import concurrent.futures
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import SwitchController


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# ── 컨트롤러 이름 정의 ────────────────────────────────────────────────────
_ARM_NORMAL  = ['xarm7_traj_controller']
_ARM_FWD     = ['xarm7_forward_controller']

_BODY_NORMAL = ['lift_controller', 'head_controller', 'tool_controller']
_BODY_FWD    = ['lift_forward_controller', 'head_forward_controller', 'tool_forward_controller']


class ControllerModeSwitcher(Node):
    """
    arm / body 두 CM 에 동시에 SwitchController 를 호출해
    NORMAL ↔ FORWARD 모드를 원자적으로 전환한다.
    """

    def __init__(self):
        super().__init__('controller_mode_switcher')

        # ── 파라미터 ─────────────────────────────────────────────────────
        self.declare_parameter('arm_cm_ns',  '/arm/controller_manager')
        self.declare_parameter('body_cm_ns', '/body/controller_manager')
        arm_cm  = self.get_parameter('arm_cm_ns').value
        body_cm = self.get_parameter('body_cm_ns').value

        # ── 상태 (기본: NORMAL) ───────────────────────────────────────────
        self._mode        = 'normal'
        self._lock        = threading.Lock()
        self._switch_lock = threading.Lock()  # 동시 전환 방지
        self._cbg         = ReentrantCallbackGroup()

        # ── switch_controller 클라이언트 ─────────────────────────────────
        self._arm_switch_cli = self.create_client(
            SwitchController,
            f'{arm_cm}/switch_controller',
            callback_group=self._cbg,
        )
        self._body_switch_cli = self.create_client(
            SwitchController,
            f'{body_cm}/switch_controller',
            callback_group=self._cbg,
        )

        # ── 토픽 구독 (switch_mode_cmd) ───────────────────────────────────
        self.create_subscription(
            Bool,
            '~/switch_mode_cmd',
            self._on_switch_cmd,
            10,
            callback_group=self._cbg,
        )

        # ── 서비스 서버 (하위 호환 유지) ──────────────────────────────────
        self.create_service(
            SetBool,
            '~/switch_mode',
            self._handle_switch,
            callback_group=self._cbg,
        )

        # ── 모드 퍼블리셔 (latched) ────────────────────────────────────────
        self._mode_pub = self.create_publisher(String, '~/mode', _LATCHED_QOS)
        self._publish_mode()

        self.get_logger().info(
            f'ControllerModeSwitcher ready. '
            f'arm_cm={arm_cm}  body_cm={body_cm}'
        )

    # ── 헬퍼 ─────────────────────────────────────────────────────────────

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _call_switch_sync(
        self,
        client: rclpy.node.Client,
        activate: list,
        deactivate: list,
        label: str,
        timeout_sec: float = 5.0,
    ) -> tuple[bool, str]:
        """
        SwitchController 를 동기적으로 호출한다.
        threading.Event + add_done_callback 패턴으로 데드락 없이 블로킹.
        """
        if not client.wait_for_service(timeout_sec=3.0):
            return False, f'[{label}] switch_controller service not available.'

        req = SwitchController.Request()
        req.activate_controllers   = activate
        req.deactivate_controllers = deactivate
        req.strictness             = SwitchController.Request.STRICT
        req.activate_asap          = False
        req.timeout.sec            = 3
        req.timeout.nanosec        = 0

        done   = threading.Event()
        result = [None]

        def _cb(future):
            result[0] = future.result()
            done.set()

        future = client.call_async(req)
        future.add_done_callback(_cb)

        if not done.wait(timeout=timeout_sec):
            return False, f'[{label}] Timeout waiting for switch_controller.'
        if result[0] is None:
            return False, f'[{label}] switch_controller returned no result.'

        ok  = result[0].ok
        msg = 'OK' if ok else 'REJECTED by controller_manager'
        return ok, f'[{label}] {msg}'

    # ── 공통 전환 로직 ───────────────────────────────────────────────────

    def _do_switch(self, use_forward: bool) -> tuple[bool, str]:
        """
        실제 컨트롤러 전환을 수행한다.
        _switch_lock 으로 동시 실행을 방지한다.
        """
        target_mode = 'forward' if use_forward else 'normal'

        with self._lock:
            if self._mode == target_mode:
                return True, f'Already in {target_mode} mode.'

        if not self._switch_lock.acquire(blocking=False):
            msg = f'Switch already in progress, ignoring [{target_mode}] request.'
            self.get_logger().warn(msg)
            return False, msg

        try:
            if use_forward:
                arm_activate,  arm_deactivate  = _ARM_FWD,     _ARM_NORMAL
                body_activate, body_deactivate = _BODY_FWD,    _BODY_NORMAL
            else:
                arm_activate,  arm_deactivate  = _ARM_NORMAL,  _ARM_FWD
                body_activate, body_deactivate = _BODY_NORMAL, _BODY_FWD

            self.get_logger().info(
                f'Switching to [{target_mode}] mode: '
                f'arm {arm_deactivate} → {arm_activate} | '
                f'body {body_deactivate} → {body_activate}'
            )

            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
                arm_future  = pool.submit(
                    self._call_switch_sync,
                    self._arm_switch_cli, arm_activate, arm_deactivate, 'arm',
                )
                body_future = pool.submit(
                    self._call_switch_sync,
                    self._body_switch_cli, body_activate, body_deactivate, 'body',
                )
                arm_ok,  arm_msg  = arm_future.result()
                body_ok, body_msg = body_future.result()

            all_ok       = arm_ok and body_ok
            combined_msg = f'{arm_msg} | {body_msg}'

            if all_ok:
                with self._lock:
                    self._mode = target_mode
                self._publish_mode()
                self.get_logger().info(f'Mode switched to [{target_mode}].')
            else:
                self.get_logger().error(f'Switch failed: {combined_msg}')

            return all_ok, combined_msg

        finally:
            self._switch_lock.release()

    # ── 토픽 핸들러 ───────────────────────────────────────────────────────

    def _on_switch_cmd(self, msg: Bool):
        """
        ~/switch_mode_cmd (std_msgs/Bool) 수신 시 호출.
        전환 작업이 블로킹이므로 별도 스레드에서 실행한다.
        """
        threading.Thread(
            target=self._do_switch,
            args=(msg.data,),
            daemon=True,
        ).start()

    # ── 서비스 핸들러 (하위 호환 유지) ────────────────────────────────────

    def _handle_switch(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        ok, msg = self._do_switch(request.data)
        response.success = ok
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControllerModeSwitcher()
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
