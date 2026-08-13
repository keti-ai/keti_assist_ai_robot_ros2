#!/usr/bin/env python3

"""
Unified motion action server (MoveJ / MoveL / MoveT / ArmTask)

MoveJ  → Pilz PTP (joint space)
MoveL  → Pilz LIN (Cartesian linear)
MoveT  → Pilz LIN (tool delta → base frame pose goal)
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
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener

from kaair_msgs.action import MoveJoint, MoveLinear, MoveTool, ArmTask

# ── ANSI 색상 유틸 ─────────────────────────────────────────────────────────
_BLUE = "\033[94m"
_RED  = "\033[91m"
_RST  = "\033[0m"

def _blue(s: str) -> str: return f"{_BLUE}{s}{_RST}"
def _red(s: str)  -> str: return f"{_RED}{s}{_RST}"

MOVE_GROUP = "arm"
MOVE_ACTION = "move_action"
FEEDBACK_HZ = 10.0

PILZ_PIPELINE = "pilz_industrial_motion_planner"
PLANNER_PTP = "PTP"
PLANNER_LIN = "LIN"

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
DEFAULT_BASE_FRAME = "arm_base"
EEF_LINK = "tool_tcp_link"

ACTION_ARM_MOVE = "kaair_worker/arm_moveJ"
ACTION_MOVEL = "kaair_worker/arm_moveL"
ACTION_MOVET = "kaair_worker/arm_moveT"
ACTION_ARM_TASK = "kaair_worker/arm_task"

APPROACH_OFFSET_Z = 0.10
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

MIN_SCALE = 0.01
MAX_SCALE = 1.0

TASK_STEP_SCALING = {
    "movej": (0.50, 0.50),
    "movel_approach": (0.25, 0.25),
    "movel_target": (0.15, 0.15),
    "movel_retreat": (0.35, 0.35),
}

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

        self.declare_parameter("default_velocity_scale", 1.0)
        self.declare_parameter("default_acceleration_scale", 0.40)

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
            self,
            MoveGroup,
            MOVE_ACTION,
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

        self._srv_arm_task = ActionServer(
            self,
            ArmTask,
            ACTION_ARM_TASK,
            execute_callback=self._execute_arm_task,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"UnifiedMotionActionServer 시작 (Pilz PTP/LIN): "
            f"{ACTION_ARM_MOVE}, {ACTION_MOVEL}, {ACTION_MOVET}, {ACTION_ARM_TASK}"
        )

    def _cancel_cb(self, _goal_handle):
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

    def _normalize_quaternion(self, qx, qy, qz, qw):
        q = np.array([qx, qy, qz, qw], dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-9:
            raise ValueError(
                "quaternion norm이 0에 가까움. "
                "qx, qy, qz, qw 값을 확인하세요."
            )
        return q / norm

    def _clamp_scale(self, value):
        return max(MIN_SCALE, min(float(value), MAX_SCALE))

    def _resolve_scale(self, requested, param_name):
        if requested is not None and float(requested) > 0.0:
            return self._clamp_scale(requested)
        default = self.get_parameter(param_name).value
        return self._clamp_scale(default)

    def _resolve_motion_scales(self, velocity_scale, acceleration_scale):
        vel = self._resolve_scale(velocity_scale, "default_velocity_scale")
        acc = self._resolve_scale(acceleration_scale, "default_acceleration_scale")
        return vel, acc

    def _resolve_task_step_scales(self, step_kind, goal_velocity_scale, goal_acceleration_scale):
        step_vel, step_acc = TASK_STEP_SCALING[step_kind]
        if goal_velocity_scale is not None and float(goal_velocity_scale) > 0.0:
            vel = self._clamp_scale(goal_velocity_scale)
        else:
            vel = self._clamp_scale(step_vel)
        if goal_acceleration_scale is not None and float(goal_acceleration_scale) > 0.0:
            acc = self._clamp_scale(goal_acceleration_scale)
        else:
            acc = self._clamp_scale(step_acc)
        return vel, acc

    # -------------------------------------------------------------------------
    # Pilz MoveGroup goal builders
    # -------------------------------------------------------------------------
    def _build_joint_constraints(self, target_joints):
        constraints = Constraints()
        for name, pos in zip(ARM_JOINTS, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _build_pose_constraints(self, target_pose, frame_id):
        constraints = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = frame_id
        pos.link_name = EEF_LINK
        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]

        region_pose = Pose()
        region_pose.position = target_pose.position
        region_pose.orientation.w = 1.0

        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [region_pose]
        pos.constraint_region = region
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header.frame_id = frame_id
        orient.link_name = EEF_LINK
        orient.orientation = target_pose.orientation
        orient.absolute_x_axis_tolerance = 0.01
        orient.absolute_y_axis_tolerance = 0.01
        orient.absolute_z_axis_tolerance = 0.01
        orient.weight = 1.0

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(orient)
        return constraints

    def _build_pilz_move_group_goal(
        self,
        planner_id,
        goal_constraints,
        plan_only,
        velocity_scale,
        acceleration_scale,
    ):
        req = MotionPlanRequest()
        req.group_name = MOVE_GROUP
        req.planner_id = planner_id
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(acceleration_scale)
        req.goal_constraints = goal_constraints
        req.pipeline_id = PILZ_PIPELINE

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = plan_only
        return goal

    def _build_pilz_joint_goal(
        self,
        target_joints,
        plan_only,
        velocity_scale,
        acceleration_scale,
    ):
        return self._build_pilz_move_group_goal(
            PLANNER_PTP,
            [self._build_joint_constraints(target_joints)],
            plan_only,
            velocity_scale,
            acceleration_scale,
        )

    def _build_pilz_pose_goal(
        self,
        target_pose,
        frame_id,
        plan_only,
        velocity_scale,
        acceleration_scale,
    ):
        return self._build_pilz_move_group_goal(
            PLANNER_LIN,
            [self._build_pose_constraints(target_pose, frame_id)],
            plan_only,
            velocity_scale,
            acceleration_scale,
        )

    def _call_move_group_goal(
        self,
        moveit_goal,
        goal_handle,
        feedback=None,
        status="",
    ):
        if not self._move_group_client.wait_for_server(timeout_sec=3.0):
            return False, "MoveGroup 액션 서버에 연결할 수 없음"

        send_future = self._move_group_client.send_goal_async(moveit_goal)

        if feedback is not None:
            feedback.status = status
            if hasattr(feedback, "current_joints"):
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

            if feedback is not None:
                if hasattr(feedback, "current_joints"):
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

    # -------------------------------------------------------------------------
    # MoveJoint (Pilz PTP)
    # -------------------------------------------------------------------------
    def _execute_move_joint(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_joint_body(goal_handle)

    def _execute_move_joint_body(self, goal_handle):
        goal = goal_handle.request
        target_joints = list(goal.target_joints)
        plan_only = bool(goal.plan_only)
        velocity_scale, acceleration_scale = self._resolve_motion_scales(
            getattr(goal, "velocity_scale", 0.0),
            getattr(goal, "acceleration_scale", 0.0),
        )

        result = MoveJoint.Result()
        feedback = MoveJoint.Feedback()

        if len(target_joints) != 7:
            result.success = False
            result.message = f"target_joints 길이가 7이 아님: {len(target_joints)}"
            result.final_joints = self._current_joints
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveJoint] {result.message}"))
            return result

        self.get_logger().info(
            f"[MoveJoint/PTP] target_joints={target_joints}, plan_only={plan_only} "
            f"vel_scale={velocity_scale:.2f} acc_scale={acceleration_scale:.2f}"
        )

        moveit_goal = self._build_pilz_joint_goal(
            target_joints, True, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(
            moveit_goal, goal_handle, feedback=feedback, status="Planning"
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
            self.get_logger().error(_red(f"[MoveJoint] {result.message}"))
            return result

        if plan_only:
            result.success = True
            result.message = "Planning 성공 (plan_only=True, 이동 생략)"
            result.final_joints = self._current_joints
            goal_handle.succeed()
            return result

        moveit_goal = self._build_pilz_joint_goal(
            target_joints, False, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(
            moveit_goal, goal_handle, feedback=feedback, status="Moving"
        )

        result.final_joints = self._current_joints
        if ok is None:
            result.success = False
            result.message = "이동 중 취소됨"
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = (
                f"이동 완료 (vel_scale={velocity_scale:.2f}, "
                f"acc_scale={acceleration_scale:.2f})"
            )
            goal_handle.succeed()
            self.get_logger().info(_blue(f"[MoveJoint] {result.message}"))
        else:
            result.success = False
            result.message = f"이동 실패: {msg}"
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveJoint] {result.message}"))

        return result

    # -------------------------------------------------------------------------
    # Pose helpers
    # -------------------------------------------------------------------------
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
            ],
            dtype=float,
        )
        cur_q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        return cur_p, cur_q

    def _make_target_pose_from_base_xyz(
        self, x, y, z, qx, qy, qz, qw, base_frame, is_relative
    ):
        cur_p, cur_q = self._get_current_tcp_pose(base_frame)
        cur_rot = R.from_quat(cur_q)
        norm_q = self._normalize_quaternion(qx, qy, qz, qw)

        if is_relative:
            target_p = cur_p + np.array([x, y, z], dtype=float)
            target_rot = cur_rot * R.from_quat(norm_q)
        else:
            target_p = np.array([x, y, z], dtype=float)
            target_rot = R.from_quat(norm_q)

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

    def _make_target_pose_from_tool_delta(self, dx, dy, dz, qx, qy, qz, qw):
        cur_p, cur_q = self._get_current_tcp_pose()
        cur_rot = R.from_quat(cur_q)

        delta_pos_tool = np.array([dx, dy, dz], dtype=float)
        delta_pos_base = cur_rot.apply(delta_pos_tool)
        target_p = cur_p + delta_pos_base

        delta_q = self._normalize_quaternion(qx, qy, qz, qw)
        target_rot = cur_rot * R.from_quat(delta_q)
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

    def _execute_pilz_lin(
        self,
        target_pose,
        frame_id,
        plan_only,
        velocity_scale,
        acceleration_scale,
        goal_handle,
        feedback=None,
    ):
        moveit_goal = self._build_pilz_pose_goal(
            target_pose, frame_id, True, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(
            moveit_goal, goal_handle, feedback=feedback, status="Planning"
        )
        if ok is None:
            return None, "Planning 중 취소됨"
        if not ok:
            return False, f"Planning 실패: {msg}"
        if plan_only:
            return True, "Planning 성공"

        moveit_goal = self._build_pilz_pose_goal(
            target_pose, frame_id, False, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(
            moveit_goal, goal_handle, feedback=feedback, status="Moving"
        )
        if ok is None:
            return None, "이동 중 취소됨"
        return ok, msg

    # -------------------------------------------------------------------------
    # MoveLinear (Pilz LIN)
    # -------------------------------------------------------------------------
    def _execute_move_linear(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_linear_body(goal_handle)

    def _execute_move_linear_body(self, goal_handle):
        goal = goal_handle.request
        result = MoveLinear.Result()
        feedback = MoveLinear.Feedback()

        x = float(goal.x)
        y = float(goal.y)
        z = float(goal.z)
        qx = float(goal.qx)
        qy = float(goal.qy)
        qz = float(goal.qz)
        qw = float(goal.qw)
        is_relative = bool(goal.is_relative)
        plan_only = bool(goal.plan_only)
        velocity_scale, acceleration_scale = self._resolve_motion_scales(
            getattr(goal, "velocity_scale", 0.0),
            getattr(goal, "acceleration_scale", 0.0),
        )

        base_frame = getattr(goal, "base_frame", "").strip() or DEFAULT_BASE_FRAME

        self.get_logger().info(
            f"[MoveLinear/LIN] frame={base_frame} "
            f"pos=[{x:.3f},{y:.3f},{z:.3f}] is_relative={is_relative} "
            f"vel_scale={velocity_scale:.2f} acc_scale={acceleration_scale:.2f}"
        )

        try:
            target_pose = self._make_target_pose_from_base_xyz(
                x, y, z, qx, qy, qz, qw, base_frame, is_relative
            )
        except Exception as e:
            result.success = False
            result.message = f"target pose 생성 실패: {e}"
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveLinear] {result.message}"))
            return result

        result.final_pose = target_pose
        ok, msg = self._execute_pilz_lin(
            target_pose,
            base_frame,
            plan_only,
            velocity_scale,
            acceleration_scale,
            goal_handle,
            feedback=feedback,
        )

        if ok is None:
            result.success = False
            result.message = msg
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = (
                f"{msg} (vel_scale={velocity_scale:.2f}, "
                f"acc_scale={acceleration_scale:.2f})"
            )
            goal_handle.succeed()
            self.get_logger().info(_blue(f"[MoveLinear] {result.message}"))
        else:
            result.success = False
            result.message = msg
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveLinear] {result.message}"))

        return result

    # -------------------------------------------------------------------------
    # MoveTool (Pilz LIN, goal in link_base)
    # -------------------------------------------------------------------------
    def _execute_move_tool(self, goal_handle):
        with self._motion_lock:
            return self._execute_move_tool_body(goal_handle)

    def _execute_move_tool_body(self, goal_handle):
        goal = goal_handle.request
        result = MoveTool.Result()
        feedback = MoveTool.Feedback()

        dx = float(goal.dx)
        dy = float(goal.dy)
        dz = float(goal.dz)
        qx = float(goal.qx)
        qy = float(goal.qy)
        qz = float(goal.qz)
        qw = float(goal.qw)
        plan_only = bool(goal.plan_only)
        velocity_scale, acceleration_scale = self._resolve_motion_scales(
            getattr(goal, "velocity_scale", 0.0),
            getattr(goal, "acceleration_scale", 0.0),
        )

        self.get_logger().info(
            f"[MoveTool/LIN] delta=[{dx:.3f},{dy:.3f},{dz:.3f}] "
            f"vel_scale={velocity_scale:.2f} acc_scale={acceleration_scale:.2f}"
        )

        try:
            target_pose = self._make_target_pose_from_tool_delta(
                dx, dy, dz, qx, qy, qz, qw
            )
        except Exception as e:
            result.success = False
            result.message = f"target pose 생성 실패: {e}"
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveTool] {result.message}"))
            return result

        result.final_pose = target_pose
        ok, msg = self._execute_pilz_lin(
            target_pose,
            BASE_FRAME,
            plan_only,
            velocity_scale,
            acceleration_scale,
            goal_handle,
            feedback=feedback,
        )

        if ok is None:
            result.success = False
            result.message = msg
            goal_handle.canceled()
        elif ok:
            result.success = True
            result.message = (
                f"{msg} (vel_scale={velocity_scale:.2f}, "
                f"acc_scale={acceleration_scale:.2f})"
            )
            goal_handle.succeed()
            self.get_logger().info(_blue(f"[MoveTool] {result.message}"))
        else:
            result.success = False
            result.message = msg
            goal_handle.abort()
            self.get_logger().error(_red(f"[MoveTool] {result.message}"))

        return result

    # -------------------------------------------------------------------------
    # ArmTask primitives
    # -------------------------------------------------------------------------
    def _do_movej(
        self,
        target_joints,
        goal_handle,
        plan_only=False,
        velocity_scale=1.0,
        acceleration_scale=1.0,
    ):
        moveit_goal = self._build_pilz_joint_goal(
            target_joints, True, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(moveit_goal, goal_handle)
        if ok is None:
            return None, "Planning 중 취소됨"
        if not ok:
            return False, f"Planning 실패: {msg}"
        if plan_only:
            return True, "Planning 성공"

        moveit_goal = self._build_pilz_joint_goal(
            target_joints, False, velocity_scale, acceleration_scale
        )
        ok, msg = self._call_move_group_goal(moveit_goal, goal_handle)
        if ok is None:
            return None, "이동 중 취소됨"
        return ok, msg

    def _do_movel(
        self,
        x, y, z,
        qx, qy, qz, qw,
        base_frame,
        is_relative,
        goal_handle,
        plan_only=False,
        velocity_scale=1.0,
        acceleration_scale=1.0,
    ):
        try:
            target_pose = self._make_target_pose_from_base_xyz(
                x, y, z, qx, qy, qz, qw, base_frame, is_relative
            )
        except Exception as e:
            return False, f"target pose 생성 실패: {e}"

        return self._execute_pilz_lin(
            target_pose,
            base_frame,
            plan_only,
            velocity_scale,
            acceleration_scale,
            goal_handle,
        )

    def _build_task_steps(
        self,
        task_type,
        x, y, z,
        qx, qy, qz, qw,
        base_frame,
        goal_velocity_scale=0.0,
        goal_acceleration_scale=0.0,
    ):
        home_vel, home_acc = self._resolve_task_step_scales(
            "movej", goal_velocity_scale, goal_acceleration_scale
        )
        approach_vel, approach_acc = self._resolve_task_step_scales(
            "movel_approach", goal_velocity_scale, goal_acceleration_scale
        )
        target_vel, target_acc = self._resolve_task_step_scales(
            "movel_target", goal_velocity_scale, goal_acceleration_scale
        )
        retreat_vel, retreat_acc = self._resolve_task_step_scales(
            "movel_retreat", goal_velocity_scale, goal_acceleration_scale
        )

        def step_home(gh, po, _j=HOME_JOINTS, _v=home_vel, _a=home_acc):
            return self._do_movej(_j, gh, po, _v, _a)

        def step_approach(
            gh, po,
            _x=x, _y=y, _z=z, _qx=qx, _qy=qy, _qz=qz, _qw=qw, _bf=base_frame,
            _v=approach_vel, _a=approach_acc,
        ):
            return self._do_movel(
                _x, _y, _z + APPROACH_OFFSET_Z,
                _qx, _qy, _qz, _qw,
                _bf, False, gh, po, _v, _a,
            )

        def step_target(
            gh, po,
            _x=x, _y=y, _z=z, _qx=qx, _qy=qy, _qz=qz, _qw=qw, _bf=base_frame,
            _v=target_vel, _a=target_acc,
        ):
            return self._do_movel(
                _x, _y, _z,
                _qx, _qy, _qz, _qw,
                _bf, False, gh, po, _v, _a,
            )

        def step_retreat(
            gh, po,
            _x=x, _y=y, _z=z, _qx=qx, _qy=qy, _qz=qz, _qw=qw, _bf=base_frame,
            _v=retreat_vel, _a=retreat_acc,
        ):
            return self._do_movel(
                _x, _y, _z + APPROACH_OFFSET_Z,
                _qx, _qy, _qz, _qw,
                _bf, False, gh, po, _v, _a,
            )

        TASK_MAP = {
            "pick": [
                ("홈 이동 (MoveJ/PTP)", step_home),
                ("접근 위치 이동 (MoveL/LIN)", step_approach),
                ("Pick 위치 이동 (MoveL/LIN)", step_target),
            ],
            "place": [
                ("홈 이동 (MoveJ/PTP)", step_home),
                ("접근 위치 이동 (MoveL/LIN)", step_approach),
                ("Place 위치 이동 (MoveL/LIN)", step_target),
            ],
            "home": [
                ("홈 이동 (MoveJ/PTP)", step_home),
            ],
            "retreat": [
                ("후퇴 위치 이동 (MoveL/LIN)", step_retreat),
                ("홈 이동 (MoveJ/PTP)", step_home),
            ],
        }

        key = task_type.strip().lower()
        if key not in TASK_MAP:
            supported = ", ".join(TASK_MAP.keys())
            raise ValueError(
                f"알 수 없는 task_type: '{task_type}'. "
                f"지원 task_type: {supported}"
            )
        return TASK_MAP[key]

    def _execute_arm_task(self, goal_handle):
        with self._motion_lock:
            return self._execute_arm_task_body(goal_handle)

    def _execute_arm_task_body(self, goal_handle):
        goal = goal_handle.request
        result = ArmTask.Result()
        feedback = ArmTask.Feedback()

        task_type = goal.task_type.strip()
        x = float(goal.x)
        y = float(goal.y)
        z = float(goal.z)
        qx = float(goal.qx)
        qy = float(goal.qy)
        qz = float(goal.qz)
        qw = float(goal.qw)
        base_frame = goal.base_frame.strip() or DEFAULT_BASE_FRAME
        plan_only = bool(goal.plan_only)
        goal_velocity_scale = float(getattr(goal, "velocity_scale", 0.0))
        goal_acceleration_scale = float(getattr(goal, "acceleration_scale", 0.0))

        self.get_logger().info(
            f"[ArmTask] task_type={task_type} pos=[{x:.3f},{y:.3f},{z:.3f}] "
            f"goal_vel_scale={goal_velocity_scale:.2f} "
            f"goal_acc_scale={goal_acceleration_scale:.2f}"
        )

        try:
            steps = self._build_task_steps(
                task_type, x, y, z, qx, qy, qz, qw, base_frame,
                goal_velocity_scale, goal_acceleration_scale,
            )
        except ValueError as e:
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        total = len(steps)
        self.get_logger().info(_blue(f"[ArmTask] 태스크 시작: {task_type} ({total}스텝)"))

        for i, (step_name, step_fn) in enumerate(steps):
            feedback.status = f"Step {i + 1}/{total}: {step_name}"
            feedback.step = i + 1
            feedback.total_steps = total
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(_blue(f"[ArmTask] {feedback.status}"))

            current_plan_only = plan_only if i == 0 else False
            ok, msg = step_fn(goal_handle, current_plan_only)

            if ok is None:
                result.success = False
                result.message = f"Step {i + 1} ({step_name}) 취소됨"
                goal_handle.canceled()
                return result

            if not ok:
                result.success = False
                result.message = f"Step {i + 1} ({step_name}) 실패: {msg}"
                goal_handle.abort()
                self.get_logger().error(_red(f"[ArmTask] {result.message}"))
                return result

            if plan_only:
                result.success = True
                result.message = f"plan_only=True — {step_name} 플래닝 성공"
                goal_handle.succeed()
                return result

        result.success = True
        result.message = f"{task_type} 완료 ({total}스텝)"
        goal_handle.succeed()
        self.get_logger().info(_blue(f"[ArmTask] {result.message}"))
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
