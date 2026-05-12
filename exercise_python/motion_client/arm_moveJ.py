#!/usr/bin/env python3
"""
MoveJoint 액션 클라이언트 → /kaair_worker/arm_moveJ

Goal: target_joints[7] (rad), plan_only (기본 false = 실행)

실행 예:
  python3 arm_moveJ.py --ros-args \\
    -p j1:=0.0 -p j2:=0.261799 -p j3:=0.0 -p j4:=0.261799 -p j5:=-3.14159 -p j6:=0.0 -p j7:=0.0 \\
    -p plan_only:=false

의존: rclpy, kaair_msgs
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kaair_msgs.action import MoveJoint


class ArmMoveJClient(Node):
    def __init__(self):
        super().__init__('arm_movej_client')
        self.declare_parameter('action_name', '/kaair_worker/arm_moveJ')
        for i in range(1, 8):
            self.declare_parameter(f'j{i}', 0.0)
        self.declare_parameter('plan_only', False)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, MoveJoint, action_name)

    def build_goal(self) -> MoveJoint.Goal:
        g = MoveJoint.Goal()
        g.target_joints = [float(self.get_parameter(f'j{i}').value) for i in range(1, 8)]
        g.plan_only = bool(self.get_parameter('plan_only').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'MoveJ goal: joints={[round(x, 4) for x in goal.target_joints]} plan_only={goal.plan_only}'
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
        if ok:
            self.get_logger().info(f'final_joints={[round(x, 4) for x in res.final_joints]}')
        return ok

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'feedback: {fb.status!r} current={[round(x, 3) for x in fb.current_joints]}',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveJClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
