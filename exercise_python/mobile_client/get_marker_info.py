#!/usr/bin/env python3
"""
/place_info (kaair_msgs/PlaceInfo) 및 장소 마커 토픽(기본: /place_markers)을 구독해
`room/place` 형식(예: living_room/shelf)에 해당하는 PlaceInfo와 RViz 마커만 출력한다.

location_server.py 와 동일하게 텍스트 마커는 `room/place` 문자열을 쓴다.

실행 예:
  python3 get_marker_info.py living_room/shelf
  python3 get_marker_info.py living_room/shelf --ros-args -p place_marker_topic:=/place_marker

의존: rclpy, kaair_msgs, visualization_msgs, geometry_msgs
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from kaair_msgs.msg import PlaceInfo


def marker_type_name(t: int) -> str:
    for name in dir(Marker):
        if not name.isupper():
            continue
        val = getattr(Marker, name, None)
        if isinstance(val, int) and val == t:
            return name
    return str(t)


def fmt_marker(m: Marker) -> str:
    lines = [
        f'  - ns={m.ns!r} id={m.id} type={marker_type_name(m.type)} action={m.action}',
        f'    frame={m.header.frame_id} pos=({m.pose.position.x:.4f}, {m.pose.position.y:.4f}, {m.pose.position.z:.4f})',
        f'    scale=({m.scale.x:.4f}, {m.scale.y:.4f}, {m.scale.z:.4f}) rgba=({m.color.r:.2f},{m.color.g:.2f},{m.color.b:.2f},{m.color.a:.2f})',
    ]
    if m.text:
        lines.append(f'    text={m.text!r}')
    return '\n'.join(lines)


def fmt_place_info(msg: PlaceInfo) -> str:
    return (
        f'PlaceInfo:\n'
        f'  room={msg.room!r} place={msg.place!r} handle={msg.handle!r}\n'
        f'  default_mode={msg.default_mode!r}\n'
        f'  placepose: dx={msg.place_dx} dy={msg.place_dy}\n'
        f'  height={msg.height} (서버에서 모바일 베이스 높이 보정값)\n'
        f'  loc: x={msg.loc_x} y={msg.loc_y} theta(deg)={msg.loc_theta}\n'
        f'  dforward={msg.dforward} dshift={msg.dshift}'
    )


class GetMarkerInfo(Node):
    def __init__(self, place_key_cli: Optional[str], follow: bool):
        super().__init__('get_marker_info')
        self.declare_parameter('place_key', '')
        self.declare_parameter('info_topic', '/place_info')
        self.declare_parameter('place_marker_topic', '/place_markers')
        self.declare_parameter('wait_sec', 30.0)

        pk = (place_key_cli or '').strip() or str(self.get_parameter('place_key').value).strip()
        if not pk:
            raise ValueError('place_key 가 비었습니다. 인자 `room/place` 또는 파라미터 place_key 를 설정하세요.')

        self._place_key = pk
        self._follow = follow
        self._done = False

        self._latest_markers: Optional[MarkerArray] = None
        self._last_place: Optional[PlaceInfo] = None

        info_topic = self.get_parameter('info_topic').value
        marker_topic = self.get_parameter('place_marker_topic').value

        self.create_subscription(PlaceInfo, info_topic, self._on_place_info, 50)
        self.create_subscription(MarkerArray, marker_topic, self._on_markers, 10)

        self.get_logger().info(
            f'구독: {info_topic}, {marker_topic} — 대상 place_key={self._place_key!r}'
        )

    def _on_place_info(self, msg: PlaceInfo):
        if f'{msg.room}/{msg.place}' != self._place_key:
            return
        self._last_place = msg
        self._try_print()

    def _on_markers(self, msg: MarkerArray):
        self._latest_markers = msg
        if self._last_place is not None:
            self._try_print()

    def _matching_markers(self) -> List[Marker]:
        if self._latest_markers is None:
            return []
        text_markers: List[Marker] = []
        for m in self._latest_markers.markers:
            if m.ns == 'place_text' and m.text == self._place_key:
                text_markers.append(m)
        if not text_markers:
            return []
        tx = text_markers[0].pose.position.x
        ty = text_markers[0].pose.position.y
        seen_ids = {m.id for m in text_markers}
        out = list(text_markers)
        for m in self._latest_markers.markers:
            if m.ns != 'place_arrow' or m.id in seen_ids:
                continue
            dx = m.pose.position.x - tx
            dy = m.pose.position.y - ty
            if dx * dx + dy * dy < 1e-8:
                out.append(m)
                seen_ids.add(m.id)
        return out

    def _try_print(self):
        if self._last_place is None or self._latest_markers is None:
            return

        markers = self._matching_markers()
        print('---')
        print(fmt_place_info(self._last_place))
        print('Markers (filtered):')
        if not markers:
            print('  (해당 place_key 텍스트 마커가 아직 없거나, 최신 MarkerArray 에 없음)')
        for m in markers:
            print(fmt_marker(m))
        print('---\n')

        if not self._follow:
            self._done = True


def main():
    parser = argparse.ArgumentParser(description='Subscribe PlaceInfo + MarkerArray for one place.')
    parser.add_argument(
        'place_key',
        nargs='?',
        default=None,
        help='예: living_room/shelf (room/place). 생략 시 파라미터 place_key 사용.',
    )
    parser.add_argument(
        '--follow',
        action='store_true',
        help='같은 키에 대한 이후 업데이트도 계속 출력',
    )
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args if ros_args else None)

    try:
        node = GetMarkerInfo(args.place_key, follow=args.follow)
    except ValueError as e:
        parser.print_help()
        print(f'\n{e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)

    wait_sec = float(node.get_parameter('wait_sec').value)
    deadline = time.monotonic() + wait_sec

    try:
        if args.follow:
            rclpy.spin(node)
        else:
            while rclpy.ok() and not node._done:
                if time.monotonic() > deadline:
                    node.get_logger().error(
                        f'{wait_sec:.0f}s 안에 {node._place_key!r} 에 대한 '
                        'PlaceInfo + MarkerArray 를 모두 받지 못했습니다.'
                    )
                    sys.exit(1)
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
