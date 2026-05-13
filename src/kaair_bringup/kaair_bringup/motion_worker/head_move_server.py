#!/usr/bin/env python3

"""MoveIt MoveGroup(head) 기반 헤드 관절 액션 서버."""

import io
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF

from kaair_msgs.action import HeadMove

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

MOVE_GROUP = "head"
HEAD_JOINT1 = "head_joint1"
HEAD_JOINT2 = "head_joint2"
HEAD_JOINTS = (HEAD_JOINT1, HEAD_JOINT2)
MOVE_ACTION = "move_action"
ACTION_HEAD = "kaair_worker/head_move"
FEEDBACK_HZ = 10.0

MOVEIT_ERROR_MAP = {
    MoveItErrorCodes.PLANNING_FAILED: "Planning 실패",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "유효하지 않은 플랜",
    MoveItErrorCodes.CONTROL_FAILED: "실행 실패 (컨트롤러 오류)",
    MoveItErrorCodes.NO_IK_SOLUTION: "IK 해 없음",
    MoveItErrorCodes.GOAL_IN_COLLISION: "목표 위치 충돌",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "시작 상태 충돌",
}


class HeadMoveActionServer(Node):
    def __init__(self):
        super().__init__("head_move_action_server")

        self._joint_lower = {j: -math.inf for j in HEAD_JOINTS}
        self._joint_upper = {j: math.inf for j in HEAD_JOINTS}
        self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            _LATCHED_QOS,
        )

        self._j1 = 0.0
        self._j2 = 0.0
        self._active_mg_handle = None

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._move_client = ActionClient(self, MoveGroup, MOVE_ACTION)

        ActionServer(
            self,
            HeadMove,
            ACTION_HEAD,
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info(f"HeadMoveActionServer 시작: /{ACTION_HEAD} (joint limit 대기 중)")

    def _on_robot_description(self, msg: String):
        try:
            _stderr, sys.stderr = sys.stderr, io.StringIO()
            robot = URDF.from_xml_string(msg.data)
            sys.stderr = _stderr
            for jname in HEAD_JOINTS:
                joint = next((j for j in robot.joints if j.name == jname), None)
                if joint and joint.limit:
                    self._joint_lower[jname] = float(joint.limit.lower)
                    self._joint_upper[jname] = float(joint.limit.upper)
                else:
                    self.get_logger().warn(f"URDF에서 {jname} 리밋을 찾지 못함 — 해당 축 검증 비활성화")
            self.get_logger().info(
                f"{HEAD_JOINT1}: [{self._joint_lower[HEAD_JOINT1]:.3f}, "
                f"{self._joint_upper[HEAD_JOINT1]:.3f}] rad, "
                f"{HEAD_JOINT2}: [{self._joint_lower[HEAD_JOINT2]:.3f}, "
                f"{self._joint_upper[HEAD_JOINT2]:.3f}] rad"
            )
        except Exception as e:
            self.get_logger().warn(f"URDF 파싱 실패 — joint limit 검증 비활성화: {e}")

    def _on_joint_state(self, msg: JointState):
        if HEAD_JOINT1 in msg.name:
            self._j1 = float(msg.position[msg.name.index(HEAD_JOINT1)])
        if HEAD_JOINT2 in msg.name:
            self._j2 = float(msg.position[msg.name.index(HEAD_JOINT2)])

    def _fill_feedback(self, fb: HeadMove.Feedback, status: str):
        fb.status = status
        fb.current_head_joint1 = self._j1
        fb.current_head_joint2 = self._j2

    def _fill_result_positions(self, res: HeadMove.Result):
        res.final_head_joint1 = self._j1
        res.final_head_joint2 = self._j2

    def _build_moveit_goal(self, j1: float, j2: float, plan_only: bool) -> MoveGroup.Goal:
        constraints = Constraints()
        for jname, pos in ((HEAD_JOINT1, j1), (HEAD_JOINT2, j2)):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(pos)
            jc.tolerance_above = 0.002
            jc.tolerance_below = 0.002
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = MOVE_GROUP
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = plan_only
        return goal

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _within_limits(self, j1: float, j2: float) -> tuple[bool, str]:
        for jname, val in ((HEAD_JOINT1, j1), (HEAD_JOINT2, j2)):
            lo, hi = self._joint_lower[jname], self._joint_upper[jname]
            if not (lo <= val <= hi):
                return False, (
                    f"{jname} 목표 {val:.4f} rad가 허용 범위를 벗어남 [{lo:.4f}, {hi:.4f}]"
                )
        return True, ""

    def _call_move_group(
        self,
        j1: float,
        j2: float,
        plan_only: bool,
        goal_handle,
        feedback: HeadMove.Feedback,
        status: str,
    ) -> tuple[bool | None, str]:
        if not self._move_client.wait_for_server(timeout_sec=3.0):
            return False, "MoveGroup 액션 서버에 연결할 수 없음"

        moveit_goal = self._build_moveit_goal(j1, j2, plan_only)
        send_future = self._move_client.send_goal_async(moveit_goal)

        self._fill_feedback(feedback, status)
        goal_handle.publish_feedback(feedback)

        deadline = time.time() + 10.0
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Cancelled"
            if time.time() > deadline:
                return False, "MoveGroup goal 수락 대기 타임아웃"
            time.sleep(0.02)

        mg_handle = send_future.result()
        if not mg_handle.accepted:
            return False, "MoveGroup goal 거절됨"

        self._active_mg_handle = mg_handle
        result_future = mg_handle.get_result_async()
        sleep_interval = 1.0 / FEEDBACK_HZ

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = mg_handle.cancel_goal_async()
                cdeadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cdeadline:
                    time.sleep(0.02)
                self._active_mg_handle = None
                return None, "Cancelled"
            self._fill_feedback(feedback, status)
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_interval)

        self._active_mg_handle = None

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"MoveGroup 액션 비정상 종료 (status={wrapped.status})"

        ec = wrapped.result.error_code.val
        if ec == MoveItErrorCodes.SUCCESS:
            return True, "성공"
        return False, MOVEIT_ERROR_MAP.get(ec, f"MoveIt 오류 코드 {ec}")

    def _execute_cb(self, goal_handle) -> HeadMove.Result:
        g = goal_handle.request
        j1 = float(g.head_joint1)
        j2 = float(g.head_joint2)
        plan_only = bool(g.plan_only)

        self.get_logger().info(
            f"Goal 수신 — head_joint1={j1:.4f}, head_joint2={j2:.4f}, plan_only={plan_only}"
        )

        result = HeadMove.Result()
        feedback = HeadMove.Feedback()

        ok_lim, lim_msg = self._within_limits(j1, j2)
        if not ok_lim:
            result.success = False
            result.message = lim_msg
            self._fill_result_positions(result)
            goal_handle.abort()
            self.get_logger().warn(lim_msg)
            return result

        ok, msg = self._call_move_group(
            j1, j2, True, goal_handle, feedback, "Planning"
        )

        if ok is None:
            result.success = False
            result.message = "Planning 중 취소됨"
            self._fill_result_positions(result)
            goal_handle.canceled()
            return result

        if not ok:
            result.success = False
            result.message = f"Planning 실패: {msg}"
            self._fill_result_positions(result)
            goal_handle.abort()
            self.get_logger().warn(result.message)
            return result

        self.get_logger().info("Planning 성공")

        if plan_only:
            result.success = True
            result.message = "Planning 성공 (plan_only=True, 이동 생략)"
            self._fill_result_positions(result)
            goal_handle.succeed()
            return result

        ok, msg = self._call_move_group(
            j1, j2, False, goal_handle, feedback, "Moving"
        )
        self._fill_result_positions(result)

        if ok is None:
            result.success = False
            result.message = "이동 중 취소됨"
            goal_handle.canceled()
            self.get_logger().info(result.message)
        elif ok:
            result.success = True
            result.message = f"이동 완료: j1={self._j1:.4f}, j2={self._j2:.4f}"
            goal_handle.succeed()
            self.get_logger().info(result.message)
        else:
            result.success = False
            result.message = f"이동 실패: {msg}"
            goal_handle.abort()
            self.get_logger().error(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadMoveActionServer()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
