#!/usr/bin/env python3
"""
random_initializer.py — 로봇 랜덤 자세 초기화 (원샷 실행 스크립트)

동작 순서:
  0) servo 모드 점검/정리
     3d_master_controller.py(SpaceMouse) 가 떠 있어서 SERVO ON 상태이고
     tool_forward_controller 가 활성화되어 있으면(= /body/tool_controller/
     gripper_cmd 가 아니라 forward_command_controller 가 그리퍼를 점유 중),
     물리 버튼 OFF 와 동일한 시퀀스(servo stop + tool_controller 복원)를
     요청한 뒤 계속 진행한다. 그렇지 않으면 아무 것도 하지 않고 건너뛴다.
  1) arm  : kaair_worker/arm_moveJ 로 고정 홈 관절각으로 MoveJ 이동
  2) lift : kaair_worker/lift_move 로 홈 높이 ± lift_range(m) 랜덤 이동
  3) head : kaair_worker/head_move 로 홈 각도 ± head_range(rad) 랜덤 이동
  4) tool : /body/tool_controller/gripper_cmd 액션으로 고정 위치로 이동
            (0번 단계에서 그리퍼 컨트롤러가 복원된 경우, 이 호출이 곧
            "기존 action 으로 그리퍼 초기화"를 겸한다)

각 단계는 서로 독립적으로 최선을 다해(best-effort) 실행되며, 실패해도
다음 단계로 계속 진행한다. 최종 종료 코드만 전체 성공 여부를 반영한다.

실행 예 (기본값 그대로):
  python3 random_initializer.py

실행 예 (범위/홈 위치 조정):
  python3 random_initializer.py --ros-args \
    -p lift_home:=0.4 -p lift_range:=0.1 \
    -p head_home_joint1:=0.0 -p head_home_joint2:=0.0 -p head_range:=0.3 \
    -p tool_position:=0.05

의존: rclpy, kaair_msgs, control_msgs, controller_manager_msgs, std_srvs
"""

import random
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool as BoolMsg
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger

from kaair_msgs.action import HeadMove, LiftMove, MoveJoint

# 3d_master_controller.py 와 동일한 토픽 이름 (그 노드 소스 참고)
_SERVO_OFF_REQUEST_TOPIC = '/servo_mode/request_off'
_SERVO_STATE_TEXT_TOPIC = '/servo_mode/state_text'
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class RandomInitializer(Node):
    def __init__(self):
        super().__init__('random_initializer')

        # ── arm: 고정 홈 관절각 (movej) ─────────────────────────────
        # arm_move_action_server.py 의 HOME_JOINTS 와 동일한 기본값
        # Left Home : [-1.5708, 0.2618, 0.0, 0.2618, -3.1416, 1.5708, 0.0]
        # Right Home: [1.5708, 0.2618, 0.0, 0.2618, -3.1416, 1.5708, 0.0]
        self.declare_parameter(
            'arm_home_joints', [-1.5708, 0.2618, 0.0, 0.2618, -3.1416, 1.5708, 0.0],
        )
        self.declare_parameter('arm_velocity_scale', 0.0)      # 0.0 = 서버 기본값
        self.declare_parameter('arm_acceleration_scale', 0.0)  # 0.0 = 서버 기본값

        # ── lift: 홈 높이 ± lift_range(m) 랜덤 ──────────────────────
        # lift_joint 의 실제 가동 범위(약 0.10~0.68m, 로봇별 상이)는 0을
        # 포함하지 않으므로 lift_home 기본값을 범위 중간값 부근으로 둔다.
        # 실제 설치 로봇에 맞춰 파라미터로 조정할 것.
        self.declare_parameter('lift_home', 0.4)
        self.declare_parameter('lift_range', 0.1)

        # ── head: 홈 각도 ± head_range(rad) 랜덤, 축마다 독립 ───────
        self.declare_parameter('head_home_joint1', -1.2)
        self.declare_parameter('head_home_joint2', -1.0)
        self.declare_parameter('head_range', 0.2)

        # ── tool(gripper): 고정 목표 위치 ───────────────────────────
        self.declare_parameter('tool_position', 0.05)
        self.declare_parameter('tool_max_effort', 50.0)

        # ── servo/controller 정리 관련 ──────────────────────────────
        self.declare_parameter('servo_node_name', 'servo_server')
        self.declare_parameter('body_cm_ns', '/body/controller_manager')
        self.declare_parameter('tool_forward_controller', 'tool_forward_controller')
        self.declare_parameter('tool_original_controller', 'tool_controller')
        self.declare_parameter('servo_state_wait_sec', 1.5)
        self.declare_parameter('controller_restore_timeout_sec', 5.0)

        # ── 액션 이름 (필요 시 재정의 가능) ──────────────────────────
        self.declare_parameter('arm_action_name', 'kaair_worker/arm_moveJ')
        self.declare_parameter('lift_action_name', '/kaair_worker/lift_move')
        self.declare_parameter('head_action_name', '/kaair_worker/head_move')
        self.declare_parameter('gripper_action_name', '/body/tool_controller/gripper_cmd')

        gp = self.get_parameter
        self._arm_home = [float(x) for x in gp('arm_home_joints').value]
        self._arm_vel = float(gp('arm_velocity_scale').value)
        self._arm_acc = float(gp('arm_acceleration_scale').value)

        self._lift_home = float(gp('lift_home').value)
        self._lift_range = abs(float(gp('lift_range').value))

        self._head_home1 = float(gp('head_home_joint1').value)
        self._head_home2 = float(gp('head_home_joint2').value)
        self._head_range = abs(float(gp('head_range').value))

        self._tool_pos = float(gp('tool_position').value)
        self._tool_effort = float(gp('tool_max_effort').value)

        self._servo_node_name = gp('servo_node_name').value
        self._body_cm_ns = gp('body_cm_ns').value
        self._tool_fwd_ctrl = gp('tool_forward_controller').value
        self._tool_orig_ctrl = gp('tool_original_controller').value
        self._servo_wait_sec = float(gp('servo_state_wait_sec').value)
        self._restore_timeout = float(gp('controller_restore_timeout_sec').value)

        # ── 액션 클라이언트 ──────────────────────────────────────────
        self._arm_client = ActionClient(self, MoveJoint, gp('arm_action_name').value)
        self._lift_client = ActionClient(self, LiftMove, gp('lift_action_name').value)
        self._head_client = ActionClient(self, HeadMove, gp('head_action_name').value)
        self._gripper_client = ActionClient(self, GripperCommand, gp('gripper_action_name').value)

        # ── servo 정리용 통신 채널 ───────────────────────────────────
        self._servo_off_pub = self.create_publisher(BoolMsg, _SERVO_OFF_REQUEST_TOPIC, 10)
        self._servo_pause_client = self.create_client(
            Trigger, f'/{self._servo_node_name}/pause_servo',
        )
        self._list_ctrl_client = self.create_client(
            ListControllers, f'{self._body_cm_ns}/list_controllers',
        )

        self._latest_state_text = None
        self.create_subscription(
            StringMsg, _SERVO_STATE_TEXT_TOPIC, self._on_state_text, _LATCHED_QOS,
        )

    def _on_state_text(self, msg: StringMsg):
        self._latest_state_text = msg.data

    # ══════════════════════════════════════════════════════════════
    # 0) servo 모드 점검/정리
    # ══════════════════════════════════════════════════════════════
    def cleanup_servo_mode(self):
        self.get_logger().info('=== [0/4] servo 상태 점검 ===')

        # (a) /servo_mode/state_text 최신값 수신 대기 (latched → 3d_master
        #     controller.py 가 실행 중이 아니면 아무것도 수신되지 않는다)
        self._latest_state_text = None
        deadline = time.monotonic() + self._servo_wait_sec
        while self._latest_state_text is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        servo_on = bool(self._latest_state_text) and 'SERVO' in self._latest_state_text
        self.get_logger().info(
            f'servo_mode/state_text = {self._latest_state_text!r} → servo_on={servo_on}'
        )

        # (b) tool_forward_controller 활성 여부 확인
        fwd_active = self._is_controller_active(self._tool_fwd_ctrl)
        self.get_logger().info(f'{self._tool_fwd_ctrl} active={fwd_active}')

        if not (servo_on and fwd_active):
            self.get_logger().info('SERVO OFF 상태입니다 — 정리 불필요, 건너뜁니다.')
            return

        self.get_logger().warn(
            '3d_master_controller 가 SERVO ON 상태입니다. '
            'servo OFF + tool_controller 복원을 요청합니다.'
        )

        # (c) 3d_master_controller.py 에게 OFF 요청
        #     (물리 버튼 OFF 와 완전히 동일한 시퀀스: stop_servo +
        #     tool_forward_controller → tool_controller 복원)
        self._servo_off_pub.publish(BoolMsg(data=True))

        # (d) defensive: 3d_master 없이 servo_server 만 켜진 구성도 있으므로
        #     servo_server 에도 직접 pause 를 요청한다 (idempotent 호출).
        if self._servo_pause_client.wait_for_service(timeout_sec=2.0):
            fut = self._servo_pause_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

        # (e) tool_controller 가 다시 active 상태가 될 때까지 대기
        deadline = time.monotonic() + self._restore_timeout
        while time.monotonic() < deadline:
            if self._is_controller_active(self._tool_orig_ctrl):
                self.get_logger().info(f'{self._tool_orig_ctrl} 복원 완료.')
                return
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().warn(
            f'{self._tool_orig_ctrl} 복원 확인 시간 초과 — 계속 진행합니다.'
        )

    def _is_controller_active(self, name: str) -> bool:
        if not self._list_ctrl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('list_controllers 서비스를 찾을 수 없습니다.')
            return False
        fut = self._list_ctrl_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        if res is None:
            return False
        for c in res.controller:
            if c.name == name:
                return c.state == 'active'
        return False

    # ══════════════════════════════════════════════════════════════
    # 1) arm: 고정 홈 관절각으로 MoveJ
    # ══════════════════════════════════════════════════════════════
    def move_arm_home(self) -> bool:
        self.get_logger().info(f'=== [1/4] arm MoveJ → home {self._arm_home} ===')
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('arm_moveJ 액션 서버를 찾을 수 없습니다.')
            return False

        goal = MoveJoint.Goal()
        goal.target_joints = list(self._arm_home)
        goal.plan_only = False
        goal.velocity_scale = self._arm_vel
        goal.acceleration_scale = self._arm_acc

        return self._send_and_wait(self._arm_client, goal, 'arm')

    # ══════════════════════════════════════════════════════════════
    # 2) lift: 홈 높이 ± lift_range 랜덤
    # ══════════════════════════════════════════════════════════════
    def move_lift_random(self) -> bool:
        target = self._lift_home + random.uniform(-self._lift_range, self._lift_range)
        self.get_logger().info(
            f'=== [2/4] lift_move → {target:.4f} m '
            f'(home={self._lift_home:.3f} ± {self._lift_range:.3f}) ==='
        )
        if not self._lift_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('lift_move 액션 서버를 찾을 수 없습니다.')
            return False

        goal = LiftMove.Goal()
        goal.target_height = float(target)
        goal.plan_only = False
        return self._send_and_wait(self._lift_client, goal, 'lift')

    # ══════════════════════════════════════════════════════════════
    # 3) head: 홈 각도 ± head_range 랜덤 (j1, j2 각각 독립)
    # ══════════════════════════════════════════════════════════════
    def move_head_random(self) -> bool:
        j1 = self._head_home1 + random.uniform(-self._head_range, self._head_range)
        j2 = self._head_home2 + random.uniform(-self._head_range, self._head_range)
        self.get_logger().info(
            f'=== [3/4] head_move → j1={j1:.4f} rad, j2={j2:.4f} rad '
            f'(home=({self._head_home1:.3f}, {self._head_home2:.3f}) '
            f'± {self._head_range:.3f}) ==='
        )
        if not self._head_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('head_move 액션 서버를 찾을 수 없습니다.')
            return False

        goal = HeadMove.Goal()
        goal.head_joint1 = float(j1)
        goal.head_joint2 = float(j2)
        goal.plan_only = False
        return self._send_and_wait(self._head_client, goal, 'head')

    # ══════════════════════════════════════════════════════════════
    # 4) tool: 고정 위치로 GripperCommand
    # ══════════════════════════════════════════════════════════════
    def move_tool_fixed(self) -> bool:
        self.get_logger().info(f'=== [4/4] gripper_cmd → position={self._tool_pos:.4f} m ===')
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('gripper_cmd 액션 서버를 찾을 수 없습니다.')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = self._tool_pos
        goal.command.max_effort = self._tool_effort

        send_future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('gripper goal 거절됨')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        if not result_future.done():
            self.get_logger().error('gripper 결과 대기 시간 초과')
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f'gripper 결과: position={res.position:.4f}m stalled={res.stalled} '
            f'reached_goal={res.reached_goal}'
        )
        return ok

    # ── 공통 helper: MoveJoint/LiftMove/HeadMove 는 동일한 result 구조 ──
    def _send_and_wait(self, client: ActionClient, goal, label: str) -> bool:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'{label} goal 거절됨')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            self.get_logger().error(f'{label} 결과 대기 시간 초과')
            gh.cancel_goal_async()
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and res.success
        self.get_logger().info(f'{label} result: success={res.success} msg={res.message!r}')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = RandomInitializer()
    ok = True
    try:
        node.cleanup_servo_mode()
        ok = node.move_arm_home() and ok
        ok = node.move_lift_random() and ok
        ok = node.move_head_random() and ok
        ok = node.move_tool_fixed() and ok
    except KeyboardInterrupt:
        node.get_logger().warn('사용자 중단 (Ctrl+C)')
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
