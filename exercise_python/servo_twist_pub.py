#!/usr/bin/env python3
"""
servo_twist_pub.py
──────────────────────────────────────────────────────────────────────
/servo_server/delta_twist_cmds 에 현재 타임스탬프를 포함한
TwistStamped 를 주기적으로 publish 한다.

사용법:
    python3 servo_twist_pub.py                        # x 방향 0.02 m/s, 5초
    python3 servo_twist_pub.py --lx 0.05 --duration 3
    python3 servo_twist_pub.py --rz 0.1 --duration 0  # 무한 (Ctrl+C 로 중지)
    python3 servo_twist_pub.py --lx 0.02 --ly -0.01 --rz 0.05

인자:
    --lx, --ly, --lz    선속도 (m/s)
    --rx, --ry, --rz    각속도 (rad/s)
    --frame             기준 프레임  (기본: link_base)
    --duration          발행 시간 [초]  (기본: 5.0 / 0 = 무한)
    --rate              발행 주기 [Hz]  (기본: 30)
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ServoTwistPub(Node):

    def __init__(self, lx, ly, lz, rx, ry, rz, frame, duration, rate):
        super().__init__('servo_twist_pub')

        self._lx, self._ly, self._lz = lx, ly, lz
        self._rx, self._ry, self._rz = rx, ry, rz
        self._frame    = frame
        self._duration = duration   # 0 = 무한
        self._period   = 1.0 / rate
        self._count    = 0

        self._pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            10,
        )

        self.get_logger().info(
            f'Publishing to /servo_server/delta_twist_cmds\n'
            f'  linear : ({lx}, {ly}, {lz}) m/s\n'
            f'  angular: ({rx}, {ry}, {rz}) rad/s\n'
            f'  frame  : {frame}\n'
            f'  rate   : {rate} Hz  |  duration: {"∞" if duration == 0 else f"{duration}s"}'
        )

        self._timer = self.create_timer(self._period, self._cb)

    def _cb(self):
        if self._duration > 0 and self._count * self._period >= self._duration:
            self.get_logger().info(f'{self._duration}s 완료. 종료합니다.')
            self._timer.cancel()
            rclpy.shutdown()
            return

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.twist.linear.x  = self._lx
        msg.twist.linear.y  = self._ly
        msg.twist.linear.z  = self._lz
        msg.twist.angular.x = self._rx
        msg.twist.angular.y = self._ry
        msg.twist.angular.z = self._rz

        self._pub.publish(msg)
        self._count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lx',       type=float, default=0.02)
    parser.add_argument('--ly',       type=float, default=0.0)
    parser.add_argument('--lz',       type=float, default=0.0)
    parser.add_argument('--rx',       type=float, default=0.0)
    parser.add_argument('--ry',       type=float, default=0.0)
    parser.add_argument('--rz',       type=float, default=0.0)
    parser.add_argument('--frame',    type=str,   default='link_eef')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--rate',     type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = ServoTwistPub(
        lx=args.lx, ly=args.ly, lz=args.lz,
        rx=args.rx, ry=args.ry, rz=args.rz,
        frame=args.frame,
        duration=args.duration,
        rate=args.rate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
