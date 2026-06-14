#!/usr/bin/env python3
"""
CreateObjectMarker 서비스 호출 예제 스크립트.

단일 픽셀(1개) 또는 다중 픽셀(2개 이상) 배열을 입력하여
3D 좌표 배열을 반환받고, 마커를 생성합니다.

사용법:
  # 단일 픽셀 (화살표 + 이름 마커 생성)
  python3 call_object_marker.py --label chair --uv 320 240

  # 다중 픽셀 (보라색 POINTS 마커 생성, 이름 없음)
  python3 call_object_marker.py --label path --uv 100 200 --uv 200 210 --uv 300 220

  # base_frame 지정 (기본값: slamware_map)
  python3 call_object_marker.py --label chair --uv 320 240 --base-frame base_link
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from kaair_msgs.srv import CreateObjectMarker


class ObjectMarkerClient(Node):
    def __init__(self):
        super().__init__('object_marker_client')
        self.cli = self.create_client(CreateObjectMarker, 'create_object_marker')

    def call(self, label: str, u_array: list, v_array: list, base_frame: str):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'Service "create_object_marker" is not available. '
                'Is the object_marker_server running?'
            )
            return None

        request = CreateObjectMarker.Request()
        request.object_label = label
        request.u_array = [int(u) for u in u_array]
        request.v_array = [int(v) for v in v_array]
        request.base_frame = base_frame

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out or failed.')
            return None

        return future.result()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Call CreateObjectMarker service',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--label', '-l',
        type=str,
        default='object',
        help='Object label (default: object)',
    )
    parser.add_argument(
        '--uv', '-p',
        nargs=2,
        metavar=('U', 'V'),
        action='append',
        required=True,
        help='Pixel coordinate as U V (can be specified multiple times)',
    )
    parser.add_argument(
        '--base-frame', '-f',
        type=str,
        default='',
        help='Target base frame (default: slamware_map)',
    )
    return parser.parse_args()


def main():
    args = parse_args()

    u_array = [int(uv[0]) for uv in args.uv]
    v_array = [int(uv[1]) for uv in args.uv]
    n = len(u_array)

    print(f'[ObjectMarkerClient] label="{args.label}", {n} point(s):')
    for i, (u, v) in enumerate(zip(u_array, v_array)):
        print(f'  [{i}] u={u}, v={v}')
    print(f'  base_frame="{args.base_frame or "slamware_map (default)"}"')
    print()

    rclpy.init()
    client = ObjectMarkerClient()

    try:
        response = client.call(args.label, u_array, v_array, args.base_frame)
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if response is None:
        sys.exit(1)

    print(f'[Result] success={response.success}, message="{response.message}"')

    if response.success:
        x_arr = response.x_array
        y_arr = response.y_array
        z_arr = response.z_array
        print(f'  Returned {len(x_arr)} 3D point(s):')
        for i, (x, y, z) in enumerate(zip(x_arr, y_arr, z_arr)):
            print(f'  [{i}] x={x:.4f}, y={y:.4f}, z={z:.4f}')

        if n == 1:
            print()
            print('  -> Arrow marker created (label shown in RViz)')
        else:
            print()
            print('  -> POINTS marker created (purple, no label)')
    else:
        print('  Service returned failure.')
        sys.exit(1)


if __name__ == '__main__':
    main()
