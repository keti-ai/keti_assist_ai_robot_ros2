#!/usr/bin/env python3

"""FollowJointTrajectory 컨트롤러 기반 헤드 관절 액션 서버."""

import io
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF

from kaair_msgs.action import HeadMove

# ── ANSI 색상 유틸 ─────────────────────────────────────────────────────────
_BLUE = "\033[94m"
_RED  = "\033[91m"
_RST  = "\033[0m"

def _blue(s: str) -> str: return f"{_BLUE}{s}{_RST}"
def _red(s: str)  -> str: return f"{_RED}{s}{_RST}"

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

HEAD_JOINT1 = "head_joint1"
HEAD_JOINT2 = "head_joint2"
HEAD_JOINTS = (HEAD_JOINT1, HEAD_JOINT2)

HEAD_CONTROLLER_ACTION = "/body/head_controller/follow_joint_trajectory"
ACTION_HEAD = "kaair_worker/head_move"
FEEDBACK_HZ = 10.0

# 최대 관절 속도 (rad/s) — URDF 정의값 3.0 rad/s 기준, 이동 시간 자동 계산에 사용
MAX_JOINT_VEL = 2.0
MIN_MOVE_DURATION = 0.3  # 최소 이동 시간 (초)


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

        self._motion_lock = threading.Lock()
        self._j1 = 0.0
        self._j2 = 0.0
        self._active_ctrl_handle = None

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._ctrl_client = ActionClient(self, FollowJointTrajectory, HEAD_CONTROLLER_ACTION)

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
                    self.get_logger().warning(f"URDF에서 {jname} 리밋을 찾지 못함 — 해당 축 검증 비활성화")
            self.get_logger().info(
                f"{HEAD_JOINT1}: [{self._joint_lower[HEAD_JOINT1]:.3f}, "
                f"{self._joint_upper[HEAD_JOINT1]:.3f}] rad, "
                f"{HEAD_JOINT2}: [{self._joint_lower[HEAD_JOINT2]:.3f}, "
                f"{self._joint_upper[HEAD_JOINT2]:.3f}] rad"
            )
        except Exception as e:
            self.get_logger().warning(f"URDF 파싱 실패 — joint limit 검증 비활성화: {e}")

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

    def _compute_duration(self, j1_target: float, j2_target: float) -> float:
        """현재 위치와 목표 위치의 최대 이동량으로 이동 시간을 산출한다."""
        delta = max(abs(j1_target - self._j1), abs(j2_target - self._j2))
        return max(delta / MAX_JOINT_VEL, MIN_MOVE_DURATION)

    def _build_trajectory_goal(self, j1: float, j2: float) -> FollowJointTrajectory.Goal:
        duration_sec = self._compute_duration(j1, j2)
        sec_int = int(duration_sec)
        nanosec = int((duration_sec - sec_int) * 1e9)

        point = JointTrajectoryPoint()
        point.positions = [j1, j2]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=sec_int, nanosec=nanosec)

        traj = JointTrajectory()
        traj.joint_names = [HEAD_JOINT1, HEAD_JOINT2]
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
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

    def _call_controller(
        self,
        j1: float,
        j2: float,
        goal_handle,
        feedback: HeadMove.Feedback,
    ) -> tuple[bool | None, str]:
        if not self._ctrl_client.wait_for_server(timeout_sec=3.0):
            return False, f"컨트롤러 액션 서버({HEAD_CONTROLLER_ACTION})에 연결할 수 없음"

        ctrl_goal = self._build_trajectory_goal(j1, j2)
        send_future = self._ctrl_client.send_goal_async(ctrl_goal)

        self._fill_feedback(feedback, "Moving")
        goal_handle.publish_feedback(feedback)

        deadline = time.time() + 10.0
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Cancelled"
            if time.time() > deadline:
                return False, "컨트롤러 goal 수락 대기 타임아웃"
            time.sleep(0.02)

        ctrl_handle = send_future.result()
        if not ctrl_handle.accepted:
            return False, "컨트롤러 goal 거절됨"

        self._active_ctrl_handle = ctrl_handle
        result_future = ctrl_handle.get_result_async()
        sleep_interval = 1.0 / FEEDBACK_HZ

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = ctrl_handle.cancel_goal_async()
                cdeadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cdeadline:
                    time.sleep(0.02)
                self._active_ctrl_handle = None
                return None, "Cancelled"
            self._fill_feedback(feedback, "Moving")
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_interval)

        self._active_ctrl_handle = None

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            err_str = wrapped.result.error_string if wrapped.result else ""
            return False, f"컨트롤러 액션 비정상 종료 (status={wrapped.status}, error='{err_str}')"

        ec = wrapped.result.error_code if wrapped.result else -1
        if ec == FollowJointTrajectory.Result.SUCCESSFUL:
            return True, "성공"
        return False, f"컨트롤러 오류 코드 {ec}: {wrapped.result.error_string}"

    def _execute_cb(self, goal_handle) -> HeadMove.Result:
        with self._motion_lock:
            return self._execute_cb_body(goal_handle)

    def _execute_cb_body(self, goal_handle) -> HeadMove.Result:
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
            self.get_logger().error(_red(lim_msg))
            return result

        if plan_only:
            result.success = True
            result.message = "Joint limit 검사 통과 (plan_only=True, 이동 생략)"
            self._fill_result_positions(result)
            goal_handle.succeed()
            self.get_logger().info(_blue(result.message))
            return result

        self.get_logger().info(_blue(f"헤드 이동 시작: j1={j1:.4f} rad, j2={j2:.4f} rad"))

        ok, msg = self._call_controller(j1, j2, goal_handle, feedback)
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
            self.get_logger().info(_blue(result.message))
        else:
            result.success = False
            result.message = f"이동 실패: {msg}"
            goal_handle.abort()
            self.get_logger().error(_red(result.message))

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
