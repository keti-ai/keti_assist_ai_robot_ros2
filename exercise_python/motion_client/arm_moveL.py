#!/usr/bin/env python3
"""
MoveLinear 액션 클라이언트 → /kaair_worker/arm_moveL

base_frame 기준 목표 위치 (m) 및 자세. rx, ry, rz 는 서버에서 Euler XYZ [deg].
plan_only 기본 false.

실행 예:
  python3 arm_moveL.py --ros-args \\
    -p x:=0.4 -p y:=0.0 -p z:=0.3 -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \\
    -p base_frame:=arm_base -p plan_only:=false

의존: rclpy, kaair_msgs
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kaair_msgs.action import MoveLinear


class ArmMoveLClient(Node):
    def __init__(self):
        super().__init__('arm_movel_client')
        self.declare_parameter('action_name', '/kaair_worker/arm_moveL')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('base_frame', 'arm_base')

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, MoveLinear, action_name)

    def build_goal(self) -> MoveLinear.Goal:
        g = MoveLinear.Goal()
        g.x = float(self.get_parameter('x').value)
        g.y = float(self.get_parameter('y').value)
        g.z = float(self.get_parameter('z').value)
        g.rx = float(self.get_parameter('rx').value)
        g.ry = float(self.get_parameter('ry').value)
        g.rz = float(self.get_parameter('rz').value)
        g.plan_only = bool(self.get_parameter('plan_only').value)
        g.base_frame = str(self.get_parameter('base_frame').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'MoveL goal: pos=({goal.x:.4f},{goal.y:.4f},{goal.z:.4f}) '
            f'rot_deg=({goal.rx:.2f},{goal.ry:.2f},{goal.rz:.2f}) '
            f'base_frame={goal.base_frame!r} plan_only={goal.plan_only}'
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
    node = ArmMoveLClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
