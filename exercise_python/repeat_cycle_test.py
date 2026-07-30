#!/usr/bin/env python3
"""
전체 동작 시퀀스 반복(무한/N회) 테스트 스크립트

Initialize (1회만 수행)
  1. MoveJ    : 지정 조인트로 이동
  2. Head Home
  3. Gripper Open
  4. Lift Home

Loop (지정 횟수만큼, 0 이면 Ctrl+C 로 멈출 때까지 무한 반복)
  1. MoveJ
  2. Lift Move Up
  3. Head Move
  4. MoveL
  5. MoveT
  6. Gripper Close
  7. MoveJ
  8. Gripper Open
  9. Head Move
  10. Lift Down

사용 전 CONFIG 섹션의 목표값(조인트, 좌표, 높이 등)을 실제 로봇에 맞게 수정하세요.

실행 예:
  python3 repeat_cycle_test.py --count 10
  python3 repeat_cycle_test.py --count 0                 # Ctrl+C 로 멈출 때까지 무한 반복
  python3 repeat_cycle_test.py --count 5 --plan-only      # 실제 이동 없이 플래닝만 검증
  python3 repeat_cycle_test.py --count 5 --stop-on-fail false

의존: rclpy, kaair_msgs, control_msgs, scipy
"""

import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from kaair_msgs.action import HeadMove, LiftMove, MoveJoint, MoveLinear, MoveTool


# =============================================================================
# CONFIG — 실제 로봇/작업에 맞게 아래 목표값을 수정하세요.
# =============================================================================

# --- Initialize ---
INIT_MOVEJ_JOINTS = [0.0,
0.26179683208465576,
0.0,
0.26179683208465576,
-3.141615629196167,
1.5707982778549194,
0.0]  # rad, 7축

HEAD_HOME = {"head_joint1": 0.0, "head_joint2": 0.0}  # rad
LIFT_HOME_HEIGHT = 0.3  # m
GRIPPER_OPEN_POS = 0.1  # m (virtual_gripper_joint upper limit)
GRIPPER_CLOSE_POS = 0.0  # m (virtual_gripper_joint lower limit)

# --- Loop ---
LOOP_MOVEJ_1_JOINTS = [0.0,
0.26179683208465576,
0.0,
0.26179683208465576,
-3.141615629196167,
1.5707982778549194,
0.0]  # rad, 7축
LIFT_UP_HEIGHT = 0.35  # m
HEAD_MOVE_1 = {"head_joint1": -1.57, "head_joint2": -0.5}  # rad

MOVEL_TARGET = {
    "x": 0.0, "y": 0.08, "z": 0.0,
    "rx": 0.0, "ry": 0.0, "rz": 0.0,  # deg, Euler XYZ (회전 변화 없음)
    "base_frame": "base_footprint",
    "is_relative": True,
}
MOVET_DELTA = {
    "dx": -0.05, "dy": 0.0, "dz": 0.08,
    "rx": 0.0, "ry": 0.0, "rz": 0.0,  # deg, Euler XYZ (TCP 기준 상대)
}

LOOP_MOVEJ_2_JOINTS = [0.0,
0.26179683208465576,
0.0,
0.26179683208465576,
-3.141615629196167,
1.5707982778549194,
0.0]  # rad, 7축
HEAD_MOVE_2 = {"head_joint1": 1.57, "head_joint2": 0.0}  # rad
LIFT_DOWN_HEIGHT = 0.25  # m

GRIPPER_EFFORT = 50.0

# 액션 서버 이름 (필요 시 --ros-args -r 로 리매핑 가능)
ACTION_MOVEJ = "/kaair_worker/arm_moveJ"
ACTION_MOVEL = "/kaair_worker/arm_moveL"
ACTION_MOVET = "/kaair_worker/arm_moveT"
ACTION_HEAD = "/kaair_worker/head_move"
ACTION_LIFT = "/kaair_worker/lift_move"
ACTION_GRIPPER = "/body/tool_controller/gripper_cmd"


def euler_to_quat(rx: float, ry: float, rz: float):
    q = R.from_euler("xyz", [rx, ry, rz], degrees=True).as_quat()  # [x, y, z, w]
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


class StepFailed(Exception):
    """시퀀스 중 한 스텝이 실패했을 때 (stop-on-fail 모드에서) 던지는 예외."""


class RepeatCycleTester(Node):
    def __init__(self, plan_only: bool, velocity_scale: float, acceleration_scale: float, timeout_sec: float):
        super().__init__("repeat_cycle_tester")
        self.plan_only = plan_only
        self.velocity_scale = velocity_scale
        self.acceleration_scale = acceleration_scale
        self.timeout_sec = timeout_sec

        self._movej_client = ActionClient(self, MoveJoint, ACTION_MOVEJ)
        self._movel_client = ActionClient(self, MoveLinear, ACTION_MOVEL)
        self._movet_client = ActionClient(self, MoveTool, ACTION_MOVET)
        self._head_client = ActionClient(self, HeadMove, ACTION_HEAD)
        self._lift_client = ActionClient(self, LiftMove, ACTION_LIFT)
        self._gripper_client = ActionClient(self, GripperCommand, ACTION_GRIPPER)

        self._active_goal_handle = None

    # -------------------------------------------------------------------
    # 서버 대기 (시작 시 한 번에 확인)
    # -------------------------------------------------------------------
    def wait_for_all_servers(self, timeout_sec: float = 10.0) -> bool:
        checks = [
            (self._movej_client, ACTION_MOVEJ),
            (self._movel_client, ACTION_MOVEL),
            (self._movet_client, ACTION_MOVET),
            (self._head_client, ACTION_HEAD),
            (self._lift_client, ACTION_LIFT),
            (self._gripper_client, ACTION_GRIPPER),
        ]
        ok = True
        for client, name in checks:
            self.get_logger().info(f"액션 서버 대기 중: {name}")
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error(f"액션 서버 연결 실패: {name}")
                ok = False
        return ok

    # -------------------------------------------------------------------
    # 공통 goal 전송/대기 헬퍼 (kaair_msgs 액션 공통: result.success / result.message)
    # -------------------------------------------------------------------
    def _send_and_wait(self, client: ActionClient, label: str, goal, feedback_cb=None):
        self.get_logger().info(f"[{label}] goal 전송")
        send_future = client.send_goal_async(goal, feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"[{label}] goal 거절됨")
            return False, "goal rejected"

        self._active_goal_handle = gh
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        self._active_goal_handle = None

        if not result_future.done():
            self.get_logger().error(f"[{label}] 결과 대기 시간 초과")
            gh.cancel_goal_async()
            return False, "timeout"

        wrapped = result_future.result()
        res = wrapped.result
        success = (wrapped.status == GoalStatus.STATUS_SUCCEEDED) and bool(getattr(res, "success", True))
        message = getattr(res, "message", "")
        self.get_logger().info(f"[{label}] result: success={success} msg={message!r}")
        return success, message

    def _feedback_cb(self, label):
        def cb(msg):
            self.get_logger().info(f"[{label}] feedback: {msg.feedback.status!r}", throttle_duration_sec=0.5)
        return cb

    # -------------------------------------------------------------------
    # 개별 액션 스텝
    # -------------------------------------------------------------------
    def move_j(self, joints, label="MoveJ"):
        g = MoveJoint.Goal()
        g.target_joints = [float(v) for v in joints]
        g.plan_only = self.plan_only
        g.velocity_scale = self.velocity_scale
        g.acceleration_scale = self.acceleration_scale
        return self._send_and_wait(self._movej_client, label, g, self._feedback_cb(label))

    def move_l(self, target: dict, label="MoveL"):
        g = MoveLinear.Goal()
        g.x, g.y, g.z = float(target["x"]), float(target["y"]), float(target["z"])
        g.qx, g.qy, g.qz, g.qw = euler_to_quat(target["rx"], target["ry"], target["rz"])
        g.is_relative = bool(target.get("is_relative", False))
        g.base_frame = str(target.get("base_frame", "arm_base"))
        g.plan_only = self.plan_only
        g.velocity_scale = self.velocity_scale
        g.acceleration_scale = self.acceleration_scale
        return self._send_and_wait(self._movel_client, label, g, self._feedback_cb(label))

    def move_t(self, delta: dict, label="MoveT"):
        g = MoveTool.Goal()
        g.dx, g.dy, g.dz = float(delta["dx"]), float(delta["dy"]), float(delta["dz"])
        g.qx, g.qy, g.qz, g.qw = euler_to_quat(delta["rx"], delta["ry"], delta["rz"])
        g.plan_only = self.plan_only
        g.velocity_scale = self.velocity_scale
        g.acceleration_scale = self.acceleration_scale
        return self._send_and_wait(self._movet_client, label, g, self._feedback_cb(label))

    def head_move(self, joints: dict, label="HeadMove"):
        g = HeadMove.Goal()
        g.head_joint1 = float(joints["head_joint1"])
        g.head_joint2 = float(joints["head_joint2"])
        g.plan_only = self.plan_only
        return self._send_and_wait(self._head_client, label, g, self._feedback_cb(label))

    def lift_move(self, height: float, label="LiftMove"):
        g = LiftMove.Goal()
        g.target_height = float(height)
        g.plan_only = self.plan_only
        return self._send_and_wait(self._lift_client, label, g, self._feedback_cb(label))

    def gripper_move(self, position: float, label="Gripper"):
        if self.plan_only:
            self.get_logger().info(f"[{label}] plan_only 모드 — 그리퍼는 실제 이동 없이 스킵")
            return True, "skipped (plan_only)"

        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(GRIPPER_EFFORT)

        self.get_logger().info(f"[{label}] goal 전송: position={position}m")
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{label}] 액션 서버 연결 실패")
            return False, "server unavailable"

        send_future = self._gripper_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"[{label}] goal 거절됨")
            return False, "goal rejected"

        self._active_goal_handle = gh
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        self._active_goal_handle = None

        if not result_future.done():
            self.get_logger().error(f"[{label}] 결과 대기 시간 초과")
            gh.cancel_goal_async()
            return False, "timeout"

        res = result_future.result().result
        # 그리퍼는 기구적 오차/마찰로 목표치(예: 0.1m)에 못 미치고 0.095m 부근에서
        # 멈추는 경우가 흔하므로, reached_goal/stalled 여부는 성공 판정에 반영하지 않고
        # 참고 로그로만 남긴다. goal 자체가 거절/타임아웃된 경우만 실패로 처리한다.
        self.get_logger().info(
            f"[{label}] result: position={res.position:.4f}m stalled={res.stalled} reached_goal={res.reached_goal} "
            f"(reached_goal/stalled 여부는 성공 판정에 사용하지 않음)"
        )
        return True, ""

    def cancel_active_goal(self):
        if self._active_goal_handle is not None:
            self.get_logger().warn("현재 진행 중인 goal 취소 요청")
            future = self._active_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


# -----------------------------------------------------------------------
# 시퀀스 실행
# -----------------------------------------------------------------------
def run_step(node: RepeatCycleTester, fn, *args, stop_on_fail: bool, label: str, pause: float, **kwargs):
    success, message = fn(*args, label=label, **kwargs)
    if not success and stop_on_fail:
        raise StepFailed(f"{label} 실패: {message}")
    if pause > 0:
        time.sleep(pause)
    return success


def run_initialize(node: RepeatCycleTester, stop_on_fail: bool, pause: float) -> bool:
    node.get_logger().info("========== [Initialize] 시작 ==========")
    run_step(node, node.move_j, INIT_MOVEJ_JOINTS, label="Init-MoveJ",
              stop_on_fail=stop_on_fail, pause=pause)
    run_step(node, node.head_move, HEAD_HOME, label="Init-HeadHome",
              stop_on_fail=stop_on_fail, pause=pause)
    run_step(node, node.gripper_move, GRIPPER_OPEN_POS, label="Init-GripperOpen",
              stop_on_fail=stop_on_fail, pause=pause)
    run_step(node, node.lift_move, LIFT_HOME_HEIGHT, label="Init-LiftHome",
              stop_on_fail=stop_on_fail, pause=pause)
    node.get_logger().info("========== [Initialize] 완료 ==========")
    return True


def run_loop_iteration(node: RepeatCycleTester, iteration: int, total: int,
                        stop_on_fail: bool, pause: float) -> bool:
    total_str = str(total) if total > 0 else "∞"
    node.get_logger().info(f"========== [Loop {iteration}/{total_str}] 시작 ==========")

    ok = True
    ok &= run_step(node, node.move_j, LOOP_MOVEJ_1_JOINTS, label="1.MoveJ",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.lift_move, LIFT_UP_HEIGHT, label="2.LiftUp",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.head_move, HEAD_MOVE_1, label="3.HeadMove",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.move_l, MOVEL_TARGET, label="4.MoveL",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.move_t, MOVET_DELTA, label="5.MoveT",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.gripper_move, GRIPPER_CLOSE_POS, label="6.GripperClose",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.move_j, LOOP_MOVEJ_2_JOINTS, label="7.MoveJ",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.gripper_move, GRIPPER_OPEN_POS, label="8.GripperOpen",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.head_move, HEAD_MOVE_2, label="9.HeadMove",
                    stop_on_fail=stop_on_fail, pause=pause)
    ok &= run_step(node, node.lift_move, LIFT_DOWN_HEIGHT, label="10.LiftDown",
                    stop_on_fail=stop_on_fail, pause=pause)

    node.get_logger().info(f"========== [Loop {iteration}/{total_str}] {'성공' if ok else '일부 실패'} ==========")
    return ok


def parse_args(argv):
    parser = argparse.ArgumentParser(description="전체 동작 시퀀스 반복 테스트")
    parser.add_argument("--count", "-n", type=int, default=1,
                         help="Loop 반복 횟수 (0 이면 Ctrl+C 로 멈출 때까지 무한 반복, 기본값 1)")
    parser.add_argument("--plan-only", action="store_true",
                         help="실제 이동 없이 플래닝/검증만 수행 (그리퍼는 스킵)")
    parser.add_argument("--velocity-scale", type=float, default=0.0,
                         help="arm 이동 속도 스케일 0.0~1.0 (0.0=서버 기본값)")
    parser.add_argument("--acceleration-scale", type=float, default=0.0,
                         help="arm 이동 가속도 스케일 0.0~1.0 (0.0=서버 기본값)")
    parser.add_argument("--pause", type=float, default=0.3,
                         help="각 스텝 사이 대기 시간(초), 기본 0.3")
    parser.add_argument("--iter-pause", type=float, default=0.5,
                         help="Loop 회차 사이 대기 시간(초), 기본 0.5")
    parser.add_argument("--timeout", type=float, default=120.0,
                         help="각 액션 결과 대기 timeout(초), 기본 120")
    parser.add_argument("--no-stop-on-fail", action="store_true",
                         help="스텝 실패 시에도 중단하지 않고 계속 진행 (기본은 실패 시 즉시 중단)")
    return parser.parse_args(argv)


def main():
    rclpy.init(args=sys.argv)
    # --ros-args 이후의 ROS 전용 인자를 제거하고 나머지를 argparse로 파싱
    cli_argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parse_args(cli_argv)
    stop_on_fail = not args.no_stop_on_fail

    node = RepeatCycleTester(
        plan_only=args.plan_only,
        velocity_scale=args.velocity_scale,
        acceleration_scale=args.acceleration_scale,
        timeout_sec=args.timeout,
    )

    success_count = 0
    fail_count = 0
    exit_code = 0

    try:
        if not node.wait_for_all_servers():
            node.get_logger().error("일부 액션 서버에 연결할 수 없습니다. 종료합니다.")
            exit_code = 1
            return

        run_initialize(node, stop_on_fail=stop_on_fail, pause=args.pause)

        i = 0
        infinite = args.count <= 0
        while infinite or i < args.count:
            i += 1
            ok = run_loop_iteration(node, i, args.count, stop_on_fail=stop_on_fail, pause=args.pause)
            if ok:
                success_count += 1
            else:
                fail_count += 1
            if args.iter_pause > 0:
                time.sleep(args.iter_pause)

    except StepFailed as e:
        node.get_logger().error(f"중단: {e}")
        fail_count += 1
        exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C 감지 — 현재 goal 취소 후 종료합니다.")
        node.cancel_active_goal()
    finally:
        node.get_logger().info(
            f"===== 테스트 종료: 성공 {success_count}회 / 실패 {fail_count}회 ====="
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
