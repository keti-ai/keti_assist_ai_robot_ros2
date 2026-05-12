#!/usr/bin/env python3
"""
Nav2 /spin_absolute 액션 클라이언트 예제 (nav2_msgs/action/Spin, 인터페이스 동일).

서버 쪽에서 target_yaw를 절대 자세(예: map 기준)로 해석하는 경우에 사용.
사전 조건: /spin_absolute 액션 서버가 떠 있어야 함.
의존: ros-<distro>-nav2-msgs, builtin-interfaces

실행 예:
  python3 spin_absolute.py --ros-args -p target_yaw:=3.14 -p time_allowance_sec:=30.0
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin


def duration_from_seconds(seconds: float) -> Duration:
    """builtin_interfaces/Duration (sec, nanosec)."""
    d = Duration()
    if seconds <= 0.0:
        return d
    d.sec = int(seconds)
    d.nanosec = int(round((seconds - d.sec) * 1e9))
    if d.nanosec >= 1_000_000_000:
        d.sec += 1
        d.nanosec -= 1_000_000_000
    return d


class SpinAbsoluteClient(Node):
    def __init__(self):
        super().__init__('spin_absolute_client')
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('time_allowance_sec', 60.0)
        self.declare_parameter('action_name', '/spin_absolute')

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._client = ActionClient(self, Spin, action_name)

    def send_goal_and_wait(self, timeout_sec: float = 120.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'액션 서버를 찾을 수 없습니다: {self._client._action_name}. '
                'bt_navigator 또는 해당 액션 서버가 실행 중인지 확인하세요.'
            )
            return False

        goal = Spin.Goal()
        goal.target_yaw = float(self.get_parameter('target_yaw').value)
        ta = float(self.get_parameter('time_allowance_sec').value)
        goal.time_allowance = duration_from_seconds(ta)

        self.get_logger().info(
            f'spin_absolute 목표: target_yaw={goal.target_yaw:.3f} rad '
            f'({math.degrees(goal.target_yaw):.1f} deg, 절대), '
            f'time_allowance={ta:.1f} s'
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

        wrapped = result_future.result()
        status = wrapped.status
        if status == 4:
            res = wrapped.result
            elapsed = res.total_elapsed_time
            self.get_logger().info(
                f'spin_absolute 성공. total_elapsed_time={elapsed.sec}.{elapsed.nanosec:09d} s'
            )
            return True
        self.get_logger().warn(f'spin_absolute 종료 status={status} (4=성공)')
        return False

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'feedback: angular_distance_traveled={fb.angular_distance_traveled:.3f} rad',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpinAbsoluteClient()
    try:
        ok = node.send_goal_and_wait()
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
