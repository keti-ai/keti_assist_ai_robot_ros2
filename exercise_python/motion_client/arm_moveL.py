#!/usr/bin/env python3
"""
MoveLinear 액션 클라이언트 → kaair_worker/arm_moveL

base_frame 기준 목표 위치 (m) 및 자세.
rx, ry, rz 는 Euler XYZ [deg] 로 입력 → 내부에서 quaternion 변환.
is_relative=true 이면 현재 TCP 위치 기준 상대 이동.
plan_only 기본 false.
velocity_scale, acceleration_scale (0.0~1.0, 0.0이면 서버 기본값)

실행 예 (절대 이동):
  python3 arm_moveL.py --ros-args \
    -p x:=0.4 -p y:=0.0 -p z:=0.3 \
    -p rx:=180.0 -p ry:=0.0 -p rz:=0.0 \
    -p base_frame:=arm_base -p is_relative:=false -p plan_only:=false

실행 예 (절대 이동, 속도/가속도 지정):
  python3 arm_moveL.py --ros-args \
    -p x:=0.4 -p y:=0.0 -p z:=0.3 \
    -p rx:=180.0 -p ry:=0.0 -p rz:=0.0 \
    -p base_frame:=arm_base -p is_relative:=false \
    -p velocity_scale:=0.2 -p acceleration_scale:=0.4 \
    -p plan_only:=false

실행 예 (상대 이동):
  python3 arm_moveL.py --ros-args \
    -p x:=0.0 -p y:=0.0 -p z:=0.05 \
    -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \
    -p is_relative:=true

실행 예 (상대 이동, 속도/가속도 지정):
  python3 arm_moveL.py --ros-args \
    -p x:=0.0 -p y:=0.0 -p z:=0.05 \
    -p rx:=0.0 -p ry:=0.0 -p rz:=0.0 \
    -p is_relative:=true \
    -p velocity_scale:=0.5 -p acceleration_scale:=0.5

의존: rclpy, kaair_msgs, scipy
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from kaair_msgs.action import MoveLinear


class ArmMoveLClient(Node):
    def __init__(self):
        super().__init__('arm_movel_client')
        self.declare_parameter('action_name', 'kaair_worker/arm_moveL')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        # Euler XYZ [deg] → 내부에서 quaternion 변환
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('is_relative', False)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('base_frame', 'arm_base')
        # 속도/가속도 스케일 (0.0~1.0). 0.0이면 서버 기본값 사용
        self.declare_parameter('velocity_scale', 0.0)
        self.declare_parameter('acceleration_scale', 0.0)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, MoveLinear, action_name)

    def build_goal(self) -> MoveLinear.Goal:
        g = MoveLinear.Goal()
        g.x = float(self.get_parameter('x').value)
        g.y = float(self.get_parameter('y').value)
        g.z = float(self.get_parameter('z').value)

        rx = float(self.get_parameter('rx').value)
        ry = float(self.get_parameter('ry').value)
        rz = float(self.get_parameter('rz').value)
        q = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_quat()  # [x, y, z, w]
        g.qx, g.qy, g.qz, g.qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        g.is_relative = bool(self.get_parameter('is_relative').value)
        g.plan_only = bool(self.get_parameter('plan_only').value)
        g.base_frame = str(self.get_parameter('base_frame').value)
        g.velocity_scale = float(self.get_parameter('velocity_scale').value)
        g.acceleration_scale = float(self.get_parameter('acceleration_scale').value)
        return g

    def run(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 대기 시간 초과.')
            return False

        goal = self.build_goal()
        self.get_logger().info(
            f'MoveL goal: pos=({goal.x:.4f},{goal.y:.4f},{goal.z:.4f}) '
            f'quat=({goal.qx:.4f},{goal.qy:.4f},{goal.qz:.4f},{goal.qw:.4f}) '
            f'is_relative={goal.is_relative} '
            f'base_frame={goal.base_frame!r} plan_only={goal.plan_only} '
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
    node = ArmMoveLClient()
    try:
        sys.exit(0 if node.run() else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
