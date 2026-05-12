#!/usr/bin/env python3
"""
MoveTool 액션 클라이언트 → /kaair_worker/arm_moveT

TCP 기준 상대 이동 (m) 및 상대 회전. rx, ry, rz 는 서버에서 Euler XYZ [deg].
plan_only 기본 false.

실행 예:
  python3 arm_moveT.py --ros-args \\
    -p dx:=0.0 -p dy:=0.0 -p dz:=0.05 -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \\
    -p plan_only:=false

의존: rclpy, kaair_msgs
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kaair_msgs.action import MoveTool


class ArmMoveTClient(Node):
    def __init__(self):
        super().__init__('arm_movet_client')
        self.declare_parameter('action_name', '/kaair_worker/arm_moveT')
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.0)
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('plan_only', False)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, MoveTool, action_name)

    def build_goal(self) -> MoveTool.Goal:
        g = MoveTool.Goal()
        g.dx = float(self.get_parameter('dx').value)
        g.dy = float(self.get_parameter('dy').value)
        g.dz = float(self.get_parameter('dz').value)
        g.rx = float(self.get_parameter('rx').value)
        g.ry = float(self.get_parameter('ry').value)
        g.rz = float(self.get_parameter('rz').value)
        g.plan_only = bool(self.get_parameter('plan_only').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'MoveT goal: delta=({goal.dx:.4f},{goal.dy:.4f},{goal.dz:.4f}) '
            f'rot_deg=({goal.rx:.2f},{goal.ry:.2f},{goal.rz:.2f}) plan_only={goal.plan_only}'
        )

        send_future = self._client.send_goal_async(goal, self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error('goal 거절')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error('결과 대기 시간 초과')
            self._client.cancel_goal_async(gh)
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = wrapped.status == 4 and res.success
        self.get_logger().info(f'result: success={res.success} msg={res.message!r}')
        if ok and res.final_pose is not None:
            p = res.final_pose.position
            self.get_logger().info(f'final_pose pos=({p.x:.4f},{p.y:.4f},{p.z:.4f})')
        return ok

    def _feedback_cb(self, msg):
        self.get_logger().info(
            f'feedback: {msg.feedback.status!r}',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveTClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
