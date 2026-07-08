#!/usr/bin/env python3
"""
MoveTool 액션 클라이언트 → kaair_worker/arm_moveT

TCP 좌표계 기준 상대 이동 (m) 및 상대 회전.
rx, ry, rz 는 Euler XYZ [deg] 로 입력 → 내부에서 quaternion 변환.
plan_only 기본 false.
velocity_scale, acceleration_scale (0.0~1.0, 0.0이면 서버 기본값)

실행 예:
  python3 arm_moveT.py --ros-args \
    -p dx:=0.0 -p dy:=0.0 -p dz:=0.05 \
    -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \
    -p plan_only:=false

실행 예 (속도/가속도 지정):
  python3 arm_moveT.py --ros-args \
    -p dx:=0.0 -p dy:=0.0 -p dz:=0.05 \
    -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \
    -p velocity_scale:=0.3 -p acceleration_scale:=0.5 \
    -p plan_only:=false

의존: rclpy, kaair_msgs, scipy
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from kaair_msgs.action import MoveTool


class ArmMoveTClient(Node):
    def __init__(self):
        super().__init__('arm_movet_client')
        self.declare_parameter('action_name', 'kaair_worker/arm_moveT')
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.0)
        # Euler XYZ [deg] → 내부에서 quaternion 변환
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('plan_only', False)
        # 속도/가속도 스케일 (0.0~1.0). 0.0이면 서버 기본값 사용
        self.declare_parameter('velocity_scale', 0.0)
        self.declare_parameter('acceleration_scale', 0.0)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, MoveTool, action_name)

    def build_goal(self) -> MoveTool.Goal:
        g = MoveTool.Goal()
        g.dx = float(self.get_parameter('dx').value)
        g.dy = float(self.get_parameter('dy').value)
        g.dz = float(self.get_parameter('dz').value)

        rx = float(self.get_parameter('rx').value)
        ry = float(self.get_parameter('ry').value)
        rz = float(self.get_parameter('rz').value)
        q = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_quat()  # [x, y, z, w]
        g.qx, g.qy, g.qz, g.qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        g.plan_only = bool(self.get_parameter('plan_only').value)
        g.velocity_scale = float(self.get_parameter('velocity_scale').value)
        g.acceleration_scale = float(self.get_parameter('acceleration_scale').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'MoveT goal: delta=({goal.dx:.4f},{goal.dy:.4f},{goal.dz:.4f}) '
            f'quat=({goal.qx:.4f},{goal.qy:.4f},{goal.qz:.4f},{goal.qw:.4f}) '
            f'plan_only={goal.plan_only} '
            f'vel_scale={goal.velocity_scale:.2f} acc_scale={goal.acceleration_scale:.2f}'
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
