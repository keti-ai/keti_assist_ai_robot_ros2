#!/usr/bin/env python3
"""
LiftMove 액션 클라이언트 → /kaair_worker/lift_move

서버는 MoveGroup 을 거치지 않고 /body/lift_controller/follow_joint_trajectory 를
직접 호출하므로 arm MoveGroup 과 동시 실행이 가능하다.

Goal:
  target_height  (m)   : lift_joint 목표 위치
  plan_only      (bool): true = joint limit 범위 검증만 수행, 이동 없음

실행 예:
  python3 lift_move.py --ros-args -p target_height:=0.4 -p plan_only:=false

의존: rclpy, kaair_msgs
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from kaair_msgs.action import LiftMove


class LiftMoveClient(Node):
    def __init__(self):
        super().__init__('lift_move_client')
        self.declare_parameter('action_name', '/kaair_worker/lift_move')
        self.declare_parameter('target_height', 0.0)
        self.declare_parameter('plan_only', False)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, LiftMove, action_name)

    def build_goal(self) -> LiftMove.Goal:
        g = LiftMove.Goal()
        g.target_height = float(self.get_parameter('target_height').value)
        g.plan_only = bool(self.get_parameter('plan_only').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'LiftMove goal: target_height={goal.target_height:.4f} m plan_only={goal.plan_only}'
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
            gh.cancel_goal_async()
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and res.success
        self.get_logger().info(
            f'result: success={res.success} msg={res.message!r} final_height={res.final_height:.4f} m'
        )
        return ok

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'feedback: {fb.status!r} current_height={fb.current_height:.4f} m',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = LiftMoveClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
