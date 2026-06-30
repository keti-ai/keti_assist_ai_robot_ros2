#!/usr/bin/env python3

"""
LiftMove 액션 서버 – /body/lift_controller/follow_joint_trajectory 직접 제어

MoveGroup 을 거치지 않으므로 arm MoveGroup 과 동시에 실행 가능.
"""

import io
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF

from kaair_msgs.action import LiftMove

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

LIFT_JOINT       = "lift_joint"
CONTROLLER_NS    = "/body/lift_controller"
FJT_ACTION       = f"{CONTROLLER_NS}/follow_joint_trajectory"
FEEDBACK_HZ      = 10.0
MIN_DURATION_SEC = 0.5   # 아주 짧은 이동도 최소 이 시간은 확보
VELOCITY_SCALING = 0.8   # URDF max_vel 의 80%만 사용 (여유 확보)


def _ros_duration(seconds: float) -> RosDuration:
    sec  = int(seconds)
    nsec = int((seconds - sec) * 1e9)
    return RosDuration(sec=sec, nanosec=nsec)


class LiftMoveActionServer(Node):
    def __init__(self):
        super().__init__("lift_move_action_server")

        # URDF에서 joint limit 비동기 로드
        self._joint_lower  = -math.inf
        self._joint_upper  =  math.inf
        self._max_velocity = 0.2          # URDF 파싱 전 기본값 (m/s)
        self.create_subscription(
            String, "/robot_description",
            self._on_robot_description,
            _LATCHED_QOS,
        )

        self._motion_lock = threading.Lock()
        self._current_height  = 0.0
        self._active_fjt_handle = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._fjt_client = ActionClient(self, FollowJointTrajectory, FJT_ACTION)

        ActionServer(
            self, LiftMove, "kaair_worker/lift_move",
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info(
            f"LiftMoveActionServer 시작: /kaair_worker/lift_move "
            f"→ {FJT_ACTION} (직접 제어 모드)"
        )

    # ── URDF 파싱 ─────────────────────────────────────────────────────────────
    def _on_robot_description(self, msg: String):
        try:
            _stderr, sys.stderr = sys.stderr, io.StringIO()
            robot = URDF.from_xml_string(msg.data)
            sys.stderr = _stderr
            joint = next((j for j in robot.joints if j.name == LIFT_JOINT), None)
            if joint and joint.limit:
                self._joint_lower  = float(joint.limit.lower)
                self._joint_upper  = float(joint.limit.upper)
                self._max_velocity = float(joint.limit.velocity)
                self.get_logger().info(
                    f"{LIFT_JOINT} 리밋: [{self._joint_lower:.3f}, {self._joint_upper:.3f}] m, "
                    f"max_vel={self._max_velocity:.3f} m/s"
                )
            else:
                self.get_logger().warning(f"URDF에서 {LIFT_JOINT} 리밋을 찾지 못함")
        except Exception as e:
            self.get_logger().warning(f"URDF 파싱 실패: {e}")

    # ── joint state 구독 ──────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        if LIFT_JOINT in msg.name:
            self._current_height = float(msg.position[msg.name.index(LIFT_JOINT)])

    # ── Cancel 콜백 ───────────────────────────────────────────────────────────
    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel 요청 수신")
        return CancelResponse.ACCEPT

    # ── 궤적 조립 ─────────────────────────────────────────────────────────────
    def _build_trajectory(self, target: float) -> tuple[JointTrajectory, float]:
        """
        현재위치 → target 의 2-point 직선 궤적을 생성한다.
        duration = distance / (max_vel * scaling), 최소 MIN_DURATION_SEC 보장.
        """
        distance     = abs(target - self._current_height)
        eff_vel      = self._max_velocity * VELOCITY_SCALING
        duration_sec = max(distance / eff_vel, MIN_DURATION_SEC) if eff_vel > 0 else MIN_DURATION_SEC

        traj = JointTrajectory()
        traj.joint_names = [LIFT_JOINT]

        # 시작점 (t=0): 현재 위치를 명시적으로 지정해 급격한 초기 이동 방지
        p0 = JointTrajectoryPoint()
        p0.positions       = [self._current_height]
        p0.velocities      = [0.0]
        p0.time_from_start = _ros_duration(0.0)

        # 목표점
        p1 = JointTrajectoryPoint()
        p1.positions       = [target]
        p1.velocities      = [0.0]
        p1.time_from_start = _ros_duration(duration_sec)

        traj.points = [p0, p1]
        return traj, duration_sec

    # ── FollowJointTrajectory 호출 헬퍼 ──────────────────────────────────────
    def _call_fjt(
        self,
        target: float,
        goal_handle,
        feedback: LiftMove.Feedback,
    ) -> tuple[bool | None, str]:
        """
        /body/lift_controller/follow_joint_trajectory 에 직접 goal 을 보낸다.
        반환값: (True, "성공") | (False, "오류 메시지") | (None, "Cancelled")
        """
        if not self._fjt_client.wait_for_server(timeout_sec=3.0):
            return False, "lift_controller FJT 서버에 연결할 수 없음"

        traj, duration_sec = self._build_trajectory(target)

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory          = traj
        fjt_goal.goal_time_tolerance = _ros_duration(2.0)

        self.get_logger().info(_blue(
            f"리프트 이동 시작: {self._current_height:.3f} → {target:.3f} m "
            f"(예상 {duration_sec:.2f}s)"
        ))

        send_future = self._fjt_client.send_goal_async(fjt_goal)

        # goal 수락 대기
        deadline = time.time() + 5.0
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Cancelled"
            if time.time() > deadline:
                return False, "FJT goal 수락 대기 타임아웃"
            time.sleep(0.02)

        fjt_handle = send_future.result()
        if not fjt_handle.accepted:
            return False, "lift_controller FJT goal 거절됨"

        self._active_fjt_handle = fjt_handle
        result_future   = fjt_handle.get_result_async()
        sleep_interval  = 1.0 / FEEDBACK_HZ

        # 결과 대기 – 피드백 발행 + cancel 감지
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel 감지 — FJT 취소 요청")
                cancel_future = fjt_handle.cancel_goal_async()
                cdeadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cdeadline:
                    time.sleep(0.02)
                self._active_fjt_handle = None
                return None, "Cancelled"
            feedback.current_height = self._current_height
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_interval)

        self._active_fjt_handle = None
        wrapped = result_future.result()

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            ec = wrapped.result.error_code if wrapped.result else -1
            return False, f"FJT 비정상 종료 (status={wrapped.status}, error_code={ec})"

        return True, "성공"

    # ── 액션 서버 실행 콜백 ───────────────────────────────────────────────────
    def _execute_cb(self, goal_handle) -> LiftMove.Result:
        with self._motion_lock:
            return self._execute_cb_body(goal_handle)

    def _execute_cb_body(self, goal_handle) -> LiftMove.Result:
        g             = goal_handle.request
        target_height = float(g.target_height)
        plan_only     = bool(g.plan_only)

        self.get_logger().info(
            f"Goal 수신 — target_height={target_height:.3f}m, plan_only={plan_only}"
        )

        result   = LiftMove.Result()
        feedback = LiftMove.Feedback()

        # 0단계: joint limit 검증
        if not (self._joint_lower <= target_height <= self._joint_upper):
            result.success      = False
            result.message      = (
                f"목표 높이 {target_height:.3f}m가 허용 범위 밖 "
                f"[{self._joint_lower:.3f}, {self._joint_upper:.3f}]m"
            )
            result.final_height = self._current_height
            goal_handle.abort()
            self.get_logger().error(_red(result.message))
            return result

        # plan_only=True: 실제 이동 없이 가능 여부만 확인 (범위 검증 통과 = OK)
        if plan_only:
            result.success      = True
            result.message      = "범위 검증 성공 (plan_only=True, 이동 생략)"
            result.final_height = self._current_height
            goal_handle.succeed()
            self.get_logger().info(_blue(result.message))
            return result

        # 1단계: 이동 실행
        feedback.status = "Moving"
        ok, msg = self._call_fjt(target_height, goal_handle, feedback)
        result.final_height = self._current_height

        if ok is None:
            result.success  = False
            result.message  = "이동 중 취소됨"
            goal_handle.canceled()
            self.get_logger().info(result.message)
        elif ok:
            result.success  = True
            result.message  = f"이동 완료: {self._current_height:.3f}m"
            goal_handle.succeed()
            self.get_logger().info(_blue(result.message))
        else:
            result.success  = False
            result.message  = f"이동 실패: {msg}"
            goal_handle.abort()
            self.get_logger().error(_red(result.message))

        return result


def main(args=None):
    rclpy.init(args=args)
    node = LiftMoveActionServer()
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
