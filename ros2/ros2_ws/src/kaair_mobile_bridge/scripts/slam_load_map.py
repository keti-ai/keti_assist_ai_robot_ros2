#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from slamware_ros_sdk.srv import SyncSetStcm

import os
from ament_index_python.packages import get_package_share_directory


class StcmMapLoaderWithPose(Node):

    def __init__(self):
        super().__init__('stcm_map_loader_with_pose')

        package_name = 'kaair_mobile_bridge'

        # 사용할 맵 파일명 (파라미터로 오버라이드 가능)
        self.declare_parameter('map_filename', 'dure.stcm')
        self.map_filename = self.get_parameter('map_filename').get_parameter_value().string_value

        self.cli = self.create_client(
            SyncSetStcm,
            '/sync_set_stcm'
        )

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                'Waiting for sync_set_stcm service...'
            )

        self.req = SyncSetStcm.Request()

        try:
            package_share_directory = get_package_share_directory(
                package_name
            )

            maps_dir = os.path.join(
                package_share_directory,
                'config'
            )

            self.map_load_path = os.path.join(
                maps_dir,
                self.map_filename
            )

            self.get_logger().info(
                f"Attempting to load map from: "
                f"{self.map_load_path}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to setup map path: {e}"
            )

            self.map_load_path = None

    def send_request(self):

        if self.map_load_path is None:
            self.get_logger().error(
                "Map load path was not determined."
            )
            return

        # .stcm 파일 읽기
        try:
            with open(self.map_load_path, "rb") as f:
                self.req.raw_stcm = f.read()

        except FileNotFoundError:
            self.get_logger().error(
                f"Map file not found: "
                f"{self.map_load_path}"
            )
            return

        except Exception as e:
            self.get_logger().error(
                f"Failed to read map file: {e}"
            )
            return

        # 초기 위치 설정
        self.req.robot_pose = Pose()

        self.req.robot_pose.position = Point(
            x=0.0,
            y=0.0,
            z=0.0
        )

        # yaw = 0
        self.req.robot_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0
        )

        # 서비스 요청
        future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is not None:
            self.get_logger().info(
                "Map and initial pose successfully set"
            )

        else:
            self.get_logger().error(
                "Failed to load map with pose"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StcmMapLoaderWithPose()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()