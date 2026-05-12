#!/usr/bin/env python3
"""
slamware_map 기준 base_footprint 위치(모바일 로봇 포즈) 조회.

tf2_ros.TransformListener 가 내부적으로 /tf 및 /tf_static 토픽을 구독해 버퍼를 채우고,
lookup_transform 으로 부모 프레임에서 자식 프레임 원점의 위치·자세를 얻는다.

의존: rclpy, tf2_ros, geometry_msgs

실행 예 (한 번 출력 후 종료):
  python3 get_pose.py

주기적으로 출력:
  python3 get_pose.py --ros-args -p periodic:=true -p rate_hz:=5.0
"""

import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import TransformStamped


def quat_to_yaw(q) -> float:
    """geometry_msgs/Quaternion → Z축 주위 yaw (라디안)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_to_pose_tuple(t: TransformStamped):
    """x, y, z, yaw_rad, yaw_deg."""
    p = t.transform.translation
    yaw = quat_to_yaw(t.transform.rotation)
    return p.x, p.y, p.z, yaw, math.degrees(yaw)


class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        self.declare_parameter('parent_frame', 'slamware_map')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('periodic', False)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('lookup_timeout_sec', 0.5)
        self.declare_parameter('wait_for_tf_sec', 15.0)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self._lookup_timeout = float(self.get_parameter('lookup_timeout_sec').value)

        self._buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        # spin_thread=False: 별도 executor 스레드를 쓰지 않음. 일회 조회 후 rclpy.shutdown() 시
        # ExternalShutdownException / InvalidHandle 를 피하려면 메인에서 spin_once 로 /tf 를 처리해야 함.
        self._listener = tf2_ros.TransformListener(self._buffer, self, spin_thread=False)

        self._periodic = self.get_parameter('periodic').get_parameter_value().bool_value
        if self._periodic:
            rate = max(0.1, float(self.get_parameter('rate_hz').value))
            self.create_timer(1.0 / rate, self._on_timer)
            self.get_logger().info(
                f'주기 조회 ({rate:.1f} Hz): {self.parent_frame} ← {self.child_frame} '
                '(TransformListener → /tf, /tf_static)'
            )
        else:
            wait_sec = float(self.get_parameter('wait_for_tf_sec').value)
            self.get_logger().info(
                f'일회 조회: {self.parent_frame} ← {self.child_frame} '
                f'(최대 {wait_sec:.0f}s 대기)'
            )

    def lookup_pose(self) -> TransformStamped:
        """parent_frame 에서 본 child_frame 원점 = 로봇 위치."""
        return self._buffer.lookup_transform(
            self.parent_frame,
            self.child_frame,
            Time(),
            timeout=Duration(seconds=self._lookup_timeout),
        )

    def _log_pose(self, t: TransformStamped):
        x, y, z, yaw_rad, yaw_deg = transform_to_pose_tuple(t)
        stamp = t.header.stamp
        self.get_logger().info(
            f'pose [{self.child_frame} in {self.parent_frame}]: '
            f'x={x:.4f} y={y:.4f} z={z:.4f} yaw={yaw_rad:.4f} rad ({yaw_deg:.2f}°) '
            f'[stamp {stamp.sec}.{stamp.nanosec:09d}]'
        )

    def _on_timer(self):
        try:
            t = self.lookup_pose()
            self._log_pose(t)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'tf lookup 실패: {e}')

    def run_once_blocking(self) -> bool:
        deadline = time.monotonic() + float(self.get_parameter('wait_for_tf_sec').value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.lookup_pose()
                self._log_pose(t)
                return True
            except tf2_ros.TransformException:
                pass
        self.get_logger().error(
            f'TF 대기 시간 초과 ({float(self.get_parameter("wait_for_tf_sec").value):.0f}s). '
            f'{self.parent_frame} → {self.child_frame} 연결·프레임 이름을 확인하세요.'
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    periodic = node._periodic
    exit_code = 0
    try:
        if periodic:
            rclpy.spin(node)
        else:
            exit_code = 0 if node.run_once_blocking() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not periodic:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
