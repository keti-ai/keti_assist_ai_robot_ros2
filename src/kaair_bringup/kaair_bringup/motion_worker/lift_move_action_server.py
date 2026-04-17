#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes,
)
from urdf_parser_py.urdf import URDF
from kaair_msgs.action import LiftMove

# robot_state_publisher가 /robot_description을 발행할 때 쓰는 QoS
# (transient_local = 나중에 구독해도 마지막 메시지를 받을 수 있음)
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

MOVE_GROUP  = 'lift'
LIFT_JOINT  = 'lift_joint'
MOVE_ACTION = 'move_action'  # MoveIt MoveGroup 액션 서버 이름
FEEDBACK_HZ = 10.0           # 실행 중 피드백 발행 주기 (Hz)

MOVEIT_ERROR_MAP = {
    MoveItErrorCodes.PLANNING_FAILED:          "Planning 실패",
    MoveItErrorCodes.INVALID_MOTION_PLAN:      "유효하지 않은 플랜",
    MoveItErrorCodes.CONTROL_FAILED:           "실행 실패 (컨트롤러 오류)",
    MoveItErrorCodes.NO_IK_SOLUTION:           "IK 해 없음",
    MoveItErrorCodes.GOAL_IN_COLLISION:        "목표 위치 충돌",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "시작 상태 충돌",
}


class LiftMoveActionServer(Node):
    def __init__(self):
        super().__init__('lift_move_action_server')

        # ── lift_joint 리밋 (/robot_description 토픽에서 비동기 수신) ─────────
        # robot_state_publisher가 transient_local QoS로 발행하므로
        # 구독 시점이 늦어도 마지막 메시지를 즉시 받는다.
        self._joint_lower = -math.inf
        self._joint_upper =  math.inf
        self.create_subscription(
            String, '/robot_description',
            self._on_robot_description,
            _LATCHED_QOS,
        )

        self._current_height   = 0.0
        self._active_mg_handle = None   # 현재 실행 중인 MoveGroup goal handle
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self._move_client = ActionClient(self, MoveGroup, MOVE_ACTION)

        ActionServer(
            self, LiftMove, 'kaair_worker/lift_move',
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info("LiftMoveActionServer 시작: /kaair_worker/lift_move (joint limit 대기 중)")

    def _on_robot_description(self, msg: String):
        """
        /robot_description 토픽(robot_state_publisher 발행)에서
        URDF를 파싱해 lift_joint position limit을 설정한다.
        """
        try:
            import io, sys
            _stderr, sys.stderr = sys.stderr, io.StringIO()   # urdf_parser_py 경고 억제
            robot = URDF.from_xml_string(msg.data)
            sys.stderr = _stderr
            joint = next((j for j in robot.joints if j.name == LIFT_JOINT), None)
            if joint and joint.limit:
                self._joint_lower = joint.limit.lower
                self._joint_upper = joint.limit.upper
                self.get_logger().info(
                    f"{LIFT_JOINT} 리밋 로드 완료: "
                    f"[{self._joint_lower:.3f}, {self._joint_upper:.3f}]m"
                )
            else:
                self.get_logger().warn(f"URDF에서 {LIFT_JOINT} 리밋을 찾지 못함 — 검증 비활성화")
        except Exception as e:
            self.get_logger().warn(f"URDF 파싱 실패 — joint limit 검증 비활성화: {e}")

    # ── Joint state 구독 ────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        if LIFT_JOINT in msg.name:
            self._current_height = msg.position[msg.name.index(LIFT_JOINT)]

    # ── MoveGroup goal 조립 ──────────────────────────────────────────────────
    def _build_moveit_goal(self, target_height: float, plan_only: bool) -> MoveGroup.Goal:
        jc = JointConstraint()
        jc.joint_name            = LIFT_JOINT
        jc.position              = float(target_height)
        jc.tolerance_above       = 0.001
        jc.tolerance_below       = 0.001
        jc.weight                = 1.0

        constraints = Constraints()
        constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name                      = MOVE_GROUP
        req.num_planning_attempts           = 3
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request                     = req
        goal.planning_options.plan_only  = plan_only
        return goal

    # ── Cancel 콜백 ─────────────────────────────────────────────────────────
    def _cancel_cb(self, goal_handle) -> CancelResponse:
        """
        클라이언트로부터 cancel 요청이 오면 즉시 수락한다.
        실제 정지는 _execute_cb 내부의 is_cancel_requested 체크가 담당한다.
        """
        self.get_logger().info("Cancel 요청 수신")
        return CancelResponse.ACCEPT

    # ── MoveGroup 동기 호출 헬퍼 ────────────────────────────────────────────
    def _call_move_group(
        self,
        target_height: float,
        plan_only: bool,
        goal_handle,
        feedback: LiftMove.Feedback,
        status: str,
    ) -> tuple[bool | None, str]:
        """
        MoveGroup 액션을 동기적으로 호출한다.
        반환값:
            (True,  "성공")      → 성공
            (False, "메시지")    → 실패
            (None,  "Cancelled") → 클라이언트 cancel 수락
        """
        if not self._move_client.wait_for_server(timeout_sec=3.0):
            return False, "MoveGroup 액션 서버에 연결할 수 없음"

        moveit_goal = self._build_moveit_goal(target_height, plan_only)
        send_future = self._move_client.send_goal_async(moveit_goal)

        feedback.status         = status
        feedback.current_height = self._current_height
        goal_handle.publish_feedback(feedback)

        # goal 수락 대기 (cancel 감지 포함)
        deadline = time.time() + 10.0
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel 감지 — goal 수락 대기 중 취소")
                return None, "Cancelled"
            if time.time() > deadline:
                return False, "MoveGroup goal 수락 대기 타임아웃"
            time.sleep(0.02)

        mg_handle = send_future.result()
        if not mg_handle.accepted:
            return False, "MoveGroup goal 거절됨"

        # 진행 중인 MoveGroup handle을 저장 → cancel 시 원격 취소에 사용
        self._active_mg_handle = mg_handle
        result_future  = mg_handle.get_result_async()
        sleep_interval = 1.0 / FEEDBACK_HZ

        # 결과 대기 — 주기적 피드백 발행 + cancel 감지
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel 감지 — MoveGroup 실행 취소 요청")
                cancel_future = mg_handle.cancel_goal_async()
                deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < deadline:
                    time.sleep(0.02)
                self._active_mg_handle = None
                return None, "Cancelled"
            feedback.current_height = self._current_height
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_interval)

        self._active_mg_handle = None

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"MoveGroup 액션 비정상 종료 (status={wrapped.status})"

        ec = wrapped.result.error_code.val
        if ec == MoveItErrorCodes.SUCCESS:
            return True, "성공"

        label = MOVEIT_ERROR_MAP.get(ec, f"MoveIt 오류 코드 {ec}")
        return False, label

    # ── 액션 서버 실행 콜백 ─────────────────────────────────────────────────
    def _execute_cb(self, goal_handle) -> LiftMove.Result:
        goal          = goal_handle.request
        target_height = goal.target_height
        plan_only     = goal.plan_only

        self.get_logger().info(
            f"Goal 수신 — target_height={target_height:.3f}m, plan_only={plan_only}"
        )

        result   = LiftMove.Result()
        feedback = LiftMove.Feedback()

        # ── 0단계: Joint limit 사전 검증 ────────────────────────────────────
        if not (self._joint_lower <= target_height <= self._joint_upper):
            result.success      = False
            result.message      = (
                f"목표 높이 {target_height:.3f}m가 허용 범위를 벗어남 "
                f"[{self._joint_lower:.3f}, {self._joint_upper:.3f}]m"
            )
            result.final_height = self._current_height
            goal_handle.abort()                          # ← ABORTED
            self.get_logger().warn(result.message)
            return result

        # ── 1단계: Planning 가능 여부 확인 ──────────────────────────────────
        ok, msg = self._call_move_group(
            target_height, plan_only=True,
            goal_handle=goal_handle, feedback=feedback, status="Planning",
        )

        if ok is None:                                   # cancel
            result.success      = False
            result.message      = "Planning 중 취소됨"
            result.final_height = self._current_height
            goal_handle.canceled()                       # ← CANCELED
            return result

        if not ok:
            result.success      = False
            result.message      = f"Planning 실패: {msg}"
            result.final_height = self._current_height
            goal_handle.abort()                          # ← ABORTED
            self.get_logger().warn(result.message)
            return result

        self.get_logger().info("Planning 성공")

        # ── 2단계: plan_only=True 이면 여기서 종료 ──────────────────────────
        if plan_only:
            result.success      = True
            result.message      = "Planning 성공 (plan_only=True, 이동 생략)"
            result.final_height = self._current_height
            goal_handle.succeed()                        # ← SUCCEEDED
            return result

        # ── 3단계: 실제 이동 실행 ────────────────────────────────────────────
        ok, msg = self._call_move_group(
            target_height, plan_only=False,
            goal_handle=goal_handle, feedback=feedback, status="Moving",
        )

        result.final_height = self._current_height

        if ok is None:                                   # cancel
            result.success  = False
            result.message  = "이동 중 취소됨"
            goal_handle.canceled()                       # ← CANCELED
            self.get_logger().info(result.message)
        elif ok:
            result.success  = True
            result.message  = f"이동 완료: {self._current_height:.3f}m"
            goal_handle.succeed()                        # ← SUCCEEDED
            self.get_logger().info(result.message)
        else:
            result.success  = False
            result.message  = f"이동 실패: {msg}"
            goal_handle.abort()                          # ← ABORTED
            self.get_logger().error(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = LiftMoveActionServer()
    # 액션 콜백이 블로킹 대기 중에도 다른 스레드가 MoveGroup 응답을 처리하도록
    # MultiThreadedExecutor 필수
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
