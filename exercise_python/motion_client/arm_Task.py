#!/usr/bin/env python3
"""
ArmTask 액션 클라이언트 → kaair_worker/arm_task

task_type 문자열로 사전 정의된 동작 시퀀스를 실행한다.

지원 task_type:
  "pick"    — MoveJ(홈) → MoveL(접근 z+offset) → MoveL(목표)
  "place"   — MoveJ(홈) → MoveL(접근 z+offset) → MoveL(목표)
  "home"    — MoveJ(홈)
  "retreat" — MoveL(목표 위 후퇴) → MoveJ(홈)

목표 자세: rx, ry, rz [deg] (Euler XYZ) → 내부에서 quaternion 변환
plan_only=true 이면 첫 스텝 플래닝만 검증하고 실행하지 않음.

실행 예 (pick):
  python3 arm_Task.py --ros-args \
    -p task_type:=pick \
    -p x:=0.4 -p y:=0.0 -p z:=0.2 \
    -p rx:=180.0 -p ry:=0.0 -p rz:=0.0 \
    -p base_frame:=arm_base

실행 예 (home):
  python3 arm_Task.py --ros-args -p task_type:=home

의존: rclpy, kaair_msgs, scipy
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from kaair_msgs.action import ArmTask


class ArmTaskClient(Node):
    def __init__(self):
        super().__init__('arm_task_client')
        self.declare_parameter('action_name', 'kaair_worker/arm_task')
        self.declare_parameter('task_type', 'home')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        # Euler XYZ [deg] → 내부에서 quaternion 변환
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('base_frame', 'arm_base')
        self.declare_parameter('plan_only', False)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, ArmTask, action_name)

    def build_goal(self) -> ArmTask.Goal:
        g = ArmTask.Goal()
        g.task_type = str(self.get_parameter('task_type').value)
        g.x = float(self.get_parameter('x').value)
        g.y = float(self.get_parameter('y').value)
        g.z = float(self.get_parameter('z').value)

        rx = float(self.get_parameter('rx').value)
        ry = float(self.get_parameter('ry').value)
        rz = float(self.get_parameter('rz').value)
        q = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_quat()  # [x, y, z, w]
        g.qx, g.qy, g.qz, g.qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        g.base_frame = str(self.get_parameter('base_frame').value)
        g.plan_only = bool(self.get_parameter('plan_only').value)
        return g

    def run(self, timeout_sec: float = 180.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'ArmTask goal: task_type={goal.task_type!r} '
            f'pos=({goal.x:.4f},{goal.y:.4f},{goal.z:.4f}) '
            f'quat=({goal.qx:.4f},{goal.qy:.4f},{goal.qz:.4f},{goal.qw:.4f}) '
            f'base_frame={goal.base_frame!r} plan_only={goal.plan_only}'
        )

        send_future = self._client.send_goal_async(goal, self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('goal 거절됨')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error('결과 대기 시간 초과')
            gh.cancel_goal_async()
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = (wrapped.status == GoalStatus.STATUS_SUCCEEDED) and res.success

        self.get_logger().info(f'result: success={res.success} msg={res.message!r}')
        return ok

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'feedback: Step {fb.step}/{fb.total_steps} — {fb.status!r}',
            throttle_duration_sec=0.3,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
