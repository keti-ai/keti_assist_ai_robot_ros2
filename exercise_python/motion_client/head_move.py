#!/usr/bin/env python3
"""
HeadMove 액션 클라이언트 → /kaair_worker/head_move

Goal: head_joint1, head_joint2 (rad), plan_only (기본 false = 실행)

실행 예:
  python3 head_move.py --ros-args -p head_joint1:=0.0 -p head_joint2:=-0.87 -p plan_only:=false

의존: rclpy, kaair_msgs
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from kaair_msgs.action import HeadMove


class HeadMoveClient(Node):
    def __init__(self):
        super().__init__("head_move_client")
        self.declare_parameter("action_name", "/kaair_worker/head_move")
        self.declare_parameter("head_joint1", 0.0)
        self.declare_parameter("head_joint2", 0.0)
        self.declare_parameter("plan_only", False)

        action_name = self.get_parameter("action_name").value
        self._client = ActionClient(self, HeadMove, action_name)

    def build_goal(self) -> HeadMove.Goal:
        g = HeadMove.Goal()
        g.head_joint1 = float(self.get_parameter("head_joint1").value)
        g.head_joint2 = float(self.get_parameter("head_joint2").value)
        g.plan_only = bool(self.get_parameter("plan_only").value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("액션 서버 대기 시간 초과.")
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f"HeadMove goal: head_joint1={goal.head_joint1:.4f} rad "
            f"head_joint2={goal.head_joint2:.4f} rad plan_only={goal.plan_only}"
        )

        send_future = self._client.send_goal_async(goal, self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("goal 거절")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error("결과 대기 시간 초과")
            self._client.cancel_goal_async(gh)
            return False

        wrapped = result_future.result()
        res = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and res.success
        self.get_logger().info(
            f"result: success={res.success} msg={res.message!r} "
            f"final j1={res.final_head_joint1:.4f} j2={res.final_head_joint2:.4f} rad"
        )
        return ok

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f"feedback: {fb.status!r} j1={fb.current_head_joint1:.4f} j2={fb.current_head_joint2:.4f} rad",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadMoveClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
