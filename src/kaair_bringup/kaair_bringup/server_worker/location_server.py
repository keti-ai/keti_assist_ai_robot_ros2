#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class LocationServer(Node):
    def __init__(self):
        super().__init__('location_server')
        self.get_logger().info("LocationServer Node 시작 (미구현)")


def main(args=None):
    rclpy.init(args=args)
    node = LocationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()