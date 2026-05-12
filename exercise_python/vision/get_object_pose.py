#!/usr/bin/env python3
"""
/object_markers (visualization_msgs/MarkerArray) 에서 객체 마커를 찾아
지정 출력 좌표계에서 xyz 를 출력한다.

object_marker_server.py 규칙:
  · marker.ns == '/object/<label>'  (예: banana → /object/banana)
  · header.frame_id == slamware_map, ARROW(id=0) 의 points[0] 가 객체 위치

사용 예:
  python3 get_object_pose.py banana
  python3 get_object_pose.py banana --output-frame slamware_map
  python3 get_object_pose.py banana -o arm_base

의존: rclpy, visualization_msgs, geometry_msgs, tf2_ros, tf2_geometry_msgs
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args

import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray


def object_ns(label: str) -> str:
    s = label.strip()
    if s.startswith('/object/'):
        return s
    return f'/object/{s}'


def marker_xyz_in_header_frame(m: Marker) -> Optional[Tuple[float, float, float]]:
    if m.action != Marker.ADD:
        return None
    if m.type == Marker.ARROW and len(m.points) >= 1:
        p0 = m.points[0]
        return (float(p0.x), float(p0.y), float(p0.z))
    # TEXT 등 (서버는 z+0.45 에 라벨)
    if m.type == Marker.TEXT_VIEW_FACING:
        return (
            float(m.pose.position.x),
            float(m.pose.position.y),
            float(m.pose.position.z) - 0.45,
        )
    return (
        float(m.pose.position.x),
        float(m.pose.position.y),
        float(m.pose.position.z),
    )


def pick_object_xyz(markers: MarkerArray, ns: str) -> Optional[Tuple[float, float, float, str]]:
    """Returns (x,y,z, source_frame_id) or None."""
    source_frame: Optional[str] = None
    arrow_xyz: Optional[Tuple[float, float, float]] = None
    fallback_xyz: Optional[Tuple[float, float, float]] = None

    for m in markers.markers:
        if m.ns != ns:
            continue
        xyz = marker_xyz_in_header_frame(m)
        if xyz is None:
            continue
        source_frame = m.header.frame_id
        if m.type == Marker.ARROW:
            arrow_xyz = xyz
        elif fallback_xyz is None:
            fallback_xyz = xyz

    if arrow_xyz is not None and source_frame:
        return (arrow_xyz[0], arrow_xyz[1], arrow_xyz[2], source_frame)
    if fallback_xyz is not None and source_frame:
        return (fallback_xyz[0], fallback_xyz[1], fallback_xyz[2], source_frame)
    return None


class GetObjectPose(Node):
    def __init__(
        self,
        object_label: str,
        output_frame: str,
        markers_topic: str,
    ):
        super().__init__('get_object_pose')
        self._ns = object_ns(object_label)
        self._output_frame = output_frame
        self._got: Optional[Tuple[float, float, float, str]] = None

        self._buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self, spin_thread=False)

        self.create_subscription(
            MarkerArray,
            markers_topic,
            self._on_markers,
            10,
        )
        self.get_logger().info(
            f'대기: {markers_topic} 에서 ns={self._ns!r} → 출력 프레임 {output_frame!r}'
        )

    def _on_markers(self, msg: MarkerArray):
        if self._got is not None:
            return
        picked = pick_object_xyz(msg, self._ns)
        if picked is None:
            return
        self._got = picked

    def try_transform(self) -> Optional[Tuple[float, float, float]]:
        if self._got is None:
            return None
        sx, sy, sz, source_frame = self._got
        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = Time().to_msg()
        ps.point.x = sx
        ps.point.y = sy
        ps.point.z = sz
        try:
            tfm = self._buffer.lookup_transform(
                self._output_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            out = do_transform_point(ps, tfm)
            return (out.point.x, out.point.y, out.point.z)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF 변환 실패 ({source_frame} → {self._output_frame}): {e}')
            return None


def main():
    argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description='Read object xyz from /object_markers and print in output frame.',
    )
    parser.add_argument(
        'object_label',
        help='예: banana → 마커 ns /object/banana',
    )
    parser.add_argument(
        '-o',
        '--output-frame',
        default='base_footprint',
        choices=['base_footprint', 'slamware_map', 'arm_base'],
        help='결과를 표현할 좌표계 (기본: base_footprint)',
    )
    parser.add_argument(
        '--markers-topic',
        default='/object_markers',
        help='MarkerArray 토픽 이름',
    )
    parser.add_argument(
        '--wait-sec',
        type=float,
        default=20.0,
        help='마커+TF 준비까지 대기 시간',
    )
    args = parser.parse_args(argv[1:])

    rclpy.init()
    node = GetObjectPose(
        args.object_label,
        args.output_frame,
        args.markers_topic,
    )
    deadline = time.monotonic() + args.wait_sec
    xyz: Optional[Tuple[float, float, float]] = None

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node._got is None:
                continue
            xyz = node.try_transform()
            if xyz is not None:
                break
        if xyz is None and node._got is not None:
            node.get_logger().error(
                f'마커는 찾았으나 TF 변환 실패: {node._got[3]!r} → {args.output_frame!r}'
            )
            sys.exit(2)
        if xyz is None:
            node.get_logger().error(
                f'{args.wait_sec:.0f}s 안에 ns={node._ns!r} 마커를 받지 못했습니다.'
            )
            sys.exit(1)

        x, y, z = xyz
        print(
            f'object={node._ns} frame={args.output_frame!r} '
            f'x={x:.6f} y={y:.6f} z={z:.6f}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
