#!/usr/bin/env python3

"""
movej_action_server.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
developer by yjy
description:
    MoveJoint 액션 서버는 MoveJoint 액션 요청을 받아 MoveJoint 액션을 수행한다.
    MoveLinear 액션 서버는 MoveLinear 액션 요청을 받아 MoveLinear 액션을 수행한다.
    MoveTool 액션 서버는 MoveTool 액션 요청을 받아 MoveTool 액션을 수행한다.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import threading
import time

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetCartesianPath
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener

from kaair_msgs.action import MoveJoint, MoveLinear, MoveTool


MOVE_GROUP = "arm"
MOVE_ACTION = "move_action"
FEEDBACK_HZ = 10.0

ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

BASE_FRAME = "link_base"
# MoveLinear: goal.base_frame이 비어 있을 때 사용 (movel_action_server.py와 동일)
DEFAULT_BASE_FRAME = "arm_base"
EEF_LINK = "tool_tcp_link"
CARTESIAN_SERVICE = "compute_cartesian_path"
EXECUTE_ACTION = "execute_trajectory"

ACTION_ARM_MOVE = "kaair_worker/arm_moveJ"
ACTION_MOVEL = "kaair_worker/arm_moveL"
ACTION_MOVET = "kaair_worker/arm_moveT"

MOVEIT_ERROR_MAP = {
    MoveItErrorCodes.PLANNING_FAILED: "Planning 실패",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "유효하지 않은 플랜",
    MoveItErrorCodes.CONTROL_FAILED: "실행 실패",
    MoveItErrorCodes.NO_IK_SOLUTION: "IK 해 없음",
    MoveItErrorCodes.GOAL_IN_COLLISION: "목표 위치 충돌",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "시작 상태 충돌",
}


class UnifiedMotionActionServer(Node):
    def __init__(self):
        super().__init__("unified_motion_action_server")

        self._cb_group = ReentrantCallbackGroup()
        self._motion_lock = threading.Lock()

        self._current_joints = [0.0] * 7

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
            callback_group=self._cb_group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._move_group_client = ActionClient(
            self, MoveGroup, MOVE_ACTION, callback_group=self._cb_group
        )
        self._cartesian_client = self.create_client(
            GetCartesianPath,
            CARTESIAN_SERVICE,
            callback_group=self._cb_group,
        )
        self._execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            EXECUTE_ACTION,
            callback_group=self._cb_group,
        )

        self._srv_move_joint = ActionServer(
            self,
            MoveJoint,
            ACTION_ARM_MOVE,
            execute_callback=self._execute_move_joint,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._srv_move_linear = ActionServer(
            self,
            MoveLinear,
            ACTION_MOVEL,
            execute_callback=self._execute_move_linear,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._srv_move_tool = ActionServer(
            self,
            MoveTool,
            ACTION_MOVET,
            execute_callback=self._execute_move_tool,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"UnifiedMotionActionServer 시작: {ACTION_ARM_MOVE}, {ACTION_MOVEL}, {ACTION_MOVET}"
        )

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _on_joint_state(self, msg: JointState):
        values = []
        for joint_name in ARM_JOINTS:
            if joint_name not in msg.name:
                return
            idx = msg.name.index(joint_name)
            values.append(msg.position[idx])
        self._current_joints = values

    def _wait_future(self, future, timeout_sec, goal_handle=None):
        start = time.time()
        while rclpy.ok() and not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False, "cancel_requested"
            if time.time() - start > timeout_sec:
                return False, "timeout"
            time.sleep(0.01)
        return True, None

    # --- MoveJoint (MoveGroup) ---
    def _build_moveit_goal(self, target_joints, plan_only):
        constraints = Constraints()
        for name, pos in zip(ARM_JOINTS, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
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

    def _call_move_group(self, target_joints, plan_only, goal_handle, feedback, status):
        if not self._move_group_client.wait_for_server(timeout_sec=3.0):
            return False, "MoveGroup 액션 서버에 연결할 수 없음"

        moveit_goal = self._build_moveit_goal(target_joints, plan_only)
        send_future = self._move_group_client.send_goal_async(moveit_goal)

        feedback.status = status
        feedback.current_joints = self._current_joints
        goal_handle.publish_feedback(feedback)

        deadline = time.time() + 10.0
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Cancelled"
            if time.time() > deadline:
                return False, "MoveGroup goal 수락 대기 타임아웃"
            time.sleep(0.02)

        mg_handle = send_future.result()
        if mg_handle is None or not mg_handle.accepted:
            return False, "MoveGroup goal 거절됨"

        result_future = mg_handle.get_result_async()
        sleep_interval = 1.0 / FEEDBACK_HZ

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = mg_handle.cancel_goal_async()
                cdeadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cdeadline:
                    time.sleep(0.02)
                return None, "Cancelled"

            feedback.current_joints = self._current_joints
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_interval)

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"MoveGroup 액션 비정상 종료 status={wrapped.status}"

        ec = wrapped.result.error_code.val
        if ec == MoveItErrorCodes.SUCCESS:
            return True, "성공"
        return False, MOVEIT_ERROR_MAP.get(ec, f"MoveIt 오류 코드 {ec}")

    def _execute_move_joint(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_joint_body(goal_handle)

    def _execute_move_joint_body(self, goal_handle):
        goal = goal_handle.request
        target_joints = list(goal.target_joints)
        plan_only = goal.plan_only

        result = MoveJoint.Result()
        feedback = MoveJoint.Feedback()

        if len(target_joints) != 7:
            result.success = False
            result.message = f"target_joints 길이가 7이 아님: {len(target_joints)}"
            result.final_joints = self._current_joints
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"[MoveJoint] target_joints={target_joints}, plan_only={plan_only}"
        )

        ok, msg = self._call_move_group(
            target_joints=target_joints,
            plan_only=True,
            goal_handle=goal_handle,
            feedback=feedback,
            status="Planning",
        )

        if ok is None:
            result.success = False
            result.message = "Planning 중 취소됨"
            result.final_joints = self._current_joints
            goal_handle.canceled()
            return result
        if not ok:
            result.success = False
            result.message = f"Planning 실패: {msg}"
            result.final_joints = self._current_joints
            goal_handle.abort()
            return result

        if plan_only:
            result.success = True
            result.message = "Planning 성공"
            result.final_joints = self._current_joints
            goal_handle.succeed()
            return result

        ok, msg = self._call_move_group(
            target_joints=target_joints,
            plan_only=False,
            goal_handle=goal_handle,
            feedback=feedback,
            status="Moving",
        )
        result.final_joints = self._current_joints

        if ok is None:
            result.success = False
            result.message = "이동 중 취소됨"
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = "이동 완료"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"이동 실패: {msg}"
            goal_handle.abort()
        return result

    # --- Cartesian (MoveLinear / MoveTool) ---
    def _get_current_tcp_pose(self, base_frame=None):
        frame = base_frame if base_frame is not None else BASE_FRAME
        trans = self.tf_buffer.lookup_transform(
            frame,
            EEF_LINK,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=3.0),
        )
        cur_p = np.array(
            [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ]
        )
        cur_q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        return cur_p, cur_q

    def _make_target_pose_from_base_xyz(self, x, y, z, rx, ry, rz, base_frame):
        """
        MoveL: x,y,z는 base_frame 기준 절대좌표 [m],
        rx,ry,rz는 현재 TCP 자세 기준 상대 회전 [rad] (고정 축 순서 'xyz', intrinsic이 아님).
        """
        cur_p, cur_q = self._get_current_tcp_pose(base_frame)
        cur_rot = R.from_quat(cur_q)
        target_p = np.array([x, y, z])
        delta_rot = R.from_euler("xyz", [rx, ry, rz], degrees=False)
        target_rot = cur_rot * delta_rot
        target_q = target_rot.as_quat()

        target_pose = Pose()
        target_pose.position.x = float(target_p[0])
        target_pose.position.y = float(target_p[1])
        target_pose.position.z = float(target_p[2])
        target_pose.orientation.x = float(target_q[0])
        target_pose.orientation.y = float(target_q[1])
        target_pose.orientation.z = float(target_q[2])
        target_pose.orientation.w = float(target_q[3])
        return target_pose

    def _make_target_pose_from_tool_delta(self, dx, dy, dz, rx, ry, rz):
        cur_p, cur_q = self._get_current_tcp_pose()
        cur_rot = R.from_quat(cur_q)
        delta_pos_tool = np.array([dx, dy, dz])
        delta_pos_base = cur_rot.apply(delta_pos_tool)
        target_p = cur_p + delta_pos_base
        delta_rot = R.from_euler("xyz", [rx, ry, rz], degrees=False)
        target_rot = cur_rot * delta_rot
        target_q = target_rot.as_quat()

        target_pose = Pose()
        target_pose.position.x = float(target_p[0])
        target_pose.position.y = float(target_p[1])
        target_pose.position.z = float(target_p[2])
        target_pose.orientation.x = float(target_q[0])
        target_pose.orientation.y = float(target_q[1])
        target_pose.orientation.z = float(target_q[2])
        target_pose.orientation.w = float(target_q[3])
        return target_pose

    def _compute_cartesian_path(self, target_pose, goal_handle, path_frame_id=None):
        """Returns (trajectory_or_none, message, canceled).

        path_frame_id: waypoint 좌표계; None이면 BASE_FRAME (MoveTool 등 기존 동작).
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            return None, "compute_cartesian_path 서비스를 찾을 수 없음", False

        req = GetCartesianPath.Request()
        req.header.frame_id = path_frame_id if path_frame_id is not None else BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = MOVE_GROUP
        req.link_name = EEF_LINK
        req.waypoints = [target_pose]
        req.max_step = 0.002
        req.jump_threshold = 1.7
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        ok, reason = self._wait_future(future, 10.0, goal_handle)
        if not ok:
            if reason == "cancel_requested":
                return None, "", True
            return None, "Cartesian path 응답 timeout", False

        res = future.result()
        if res is None:
            return None, "Cartesian path 결과 없음", False
        if res.fraction < 0.8:
            return None, f"Cartesian path 실패 fraction={res.fraction:.3f}", False
        return res.solution, f"Cartesian path 성공 fraction={res.fraction:.3f}", False

    def _execute_trajectory(self, trajectory, goal_handle):
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            return False, False, "execute_trajectory 액션 서버를 찾을 수 없음"

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._execute_client.send_goal_async(goal)
        ok, reason = self._wait_future(send_future, 5.0, goal_handle)
        if not ok:
            if reason == "cancel_requested":
                return False, True, None
            return False, False, "ExecuteTrajectory goal 전송 timeout"

        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, False, "ExecuteTrajectory goal 거절됨"

        result_future = handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = handle.cancel_goal_async()
                deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < deadline:
                    time.sleep(0.02)
                return False, True, None
            time.sleep(0.02)

        wrapped = result_future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return True, False, "이동 완료"
        return False, False, f"ExecuteTrajectory 실패 status={wrapped.status}"

    def _execute_move_linear(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_linear_body(goal_handle)

    def _execute_move_linear_body(self, goal_handle):
        goal = goal_handle.request
        result = MoveLinear.Result()
        feedback = MoveLinear.Feedback()

        x, y, z = float(goal.x), float(goal.y), float(goal.z)
        rx = float(getattr(goal, "rx", 0.0))
        ry = float(getattr(goal, "ry", 0.0))
        rz = float(getattr(goal, "rz", 0.0))
        plan_only = bool(goal.plan_only)

        base_frame = getattr(goal, "base_frame", "").strip()
        if base_frame == "":
            base_frame = DEFAULT_BASE_FRAME

        self.get_logger().info(
            f"[MoveLinear] frame={base_frame} pos=[{x:.3f},{y:.3f},{z:.3f}] "
            f"rad rx,ry,rz=[{rx:.3f},{ry:.3f},{rz:.3f}] plan_only={plan_only}"
        )

        try:
            target_pose = self._make_target_pose_from_base_xyz(
                x, y, z, rx, ry, rz, base_frame
            )
        except Exception as e:
            result.success = False
            result.message = f"현재 TCP pose 읽기 또는 target pose 생성 실패: {e}"
            goal_handle.abort()
            return result

        result.final_pose = target_pose
        feedback.status = "Planning"
        goal_handle.publish_feedback(feedback)

        trajectory, msg, canceled = self._compute_cartesian_path(
            target_pose, goal_handle, path_frame_id=base_frame
        )
        if canceled:
            result.success = False
            result.message = "Planning 중 취소됨"
            goal_handle.canceled()
            return result
        if trajectory is None:
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        if plan_only:
            result.success = True
            result.message = msg + " / plan_only=True"
            goal_handle.succeed()
            return result

        feedback.status = "Moving"
        goal_handle.publish_feedback(feedback)

        ok, canceled, exec_msg = self._execute_trajectory(trajectory, goal_handle)
        if canceled:
            result.success = False
            result.message = "이동 중 취소됨"
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = exec_msg
            goal_handle.succeed()
        else:
            result.success = False
            result.message = exec_msg
            goal_handle.abort()
        return result

    def _execute_move_tool(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_tool_body(goal_handle)

    def _execute_move_tool_body(self, goal_handle):
        goal = goal_handle.request
        result = MoveTool.Result()
        feedback = MoveTool.Feedback()

        dx, dy, dz = float(goal.dx), float(goal.dy), float(goal.dz)
        rx = float(getattr(goal, "rx", 0.0))
        ry = float(getattr(goal, "ry", 0.0))
        rz = float(getattr(goal, "rz", 0.0))
        plan_only = bool(goal.plan_only)

        self.get_logger().info(
            f"[MoveTool] Tool delta pos=[{dx:.3f},{dy:.3f},{dz:.3f}] "
            f"rad=[{rx:.3f},{ry:.3f},{rz:.3f}] plan_only={plan_only}"
        )

        try:
            target_pose = self._make_target_pose_from_tool_delta(dx, dy, dz, rx, ry, rz)
        except Exception as e:
            result.success = False
            result.message = f"현재 TCP pose 읽기 또는 변환 실패: {e}"
            goal_handle.abort()
            return result

        result.final_pose = target_pose
        feedback.status = "Planning"
        goal_handle.publish_feedback(feedback)

        trajectory, msg, canceled = self._compute_cartesian_path(target_pose, goal_handle)
        if canceled:
            result.success = False
            result.message = "Planning 중 취소됨"
            goal_handle.canceled()
            return result
        if trajectory is None:
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        if plan_only:
            result.success = True
            result.message = msg + " / plan_only=True"
            goal_handle.succeed()
            return result

        feedback.status = "Moving"
        goal_handle.publish_feedback(feedback)

        ok, canceled, exec_msg = self._execute_trajectory(trajectory, goal_handle)
        if canceled:
            result.success = False
            result.message = "이동 중 취소됨"
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = exec_msg
            goal_handle.succeed()
        else:
            result.success = False
            result.message = exec_msg
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedMotionActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
