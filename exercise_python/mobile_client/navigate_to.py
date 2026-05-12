#!/usr/bin/env python3
"""
Nav2 /navigate_to_pose 액션 클라이언트 예제.

사전 조건: bt_navigator 등이 떠 있어 /navigate_to_pose 액션 서버가 있어야 함.
의존: ros-<distro>-nav2-msgs, geometry-msgs

실행 예:
  ros2 run <your_pkg> navigate_to.py
  python3 navigate_to.py --ros-args -p x:=1.0 -p y:=0.5 -p yaw:=0.0 -p frame_id:=map
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'slamware_map')
        self.declare_parameter('action_name', '/navigate_to_pose')

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._client = ActionClient(self, NavigateToPose, action_name)

    def send_goal_and_wait(self, timeout_sec: float = 60.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'액션 서버를 찾을 수 없습니다: {self._client._action_name}. '
                'Nav2(bt_navigator)가 실행 중인지 확인하세요.'
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('x').value)
        goal.pose.pose.position.y = float(self.get_parameter('y').value)
        goal.pose.pose.position.z = float(self.get_parameter('z').value)
        yaw = float(self.get_parameter('yaw').value)
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.get_logger().info(
            f'목표 전송: frame={goal.pose.header.frame_id} '
            f'x={goal.pose.pose.position.x:.3f} y={goal.pose.pose.position.y:.3f} yaw={yaw:.3f}'
        )

        send_future = self._client.send_goal_async(goal, self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거절되었습니다.')
            return False

        self.get_logger().info('목표 수락됨, 결과 대기 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error('결과 대기 시간 초과.')
            self._client.cancel_goal_async(goal_handle)
            return False

        status = result_future.result().status
        # action_msgs/goal status: 4=SUCCEEDED
        if status == 4:
            self.get_logger().info('내비게이션 성공.')
            return True
        self.get_logger().warn(f'내비게이션 종료 status={status} (4=성공)')
        return False

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'feedback: dist_remaining={fb.distance_remaining:.2f} m',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    try:
        ok = node.send_goal_and_wait()
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
