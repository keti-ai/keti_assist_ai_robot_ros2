#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

from slamware_ros_sdk.srv import SyncGetStcm


class StcmMapSaver(Node):

    def __init__(self):
        super().__init__('stcm_map_saver')

        package_name = 'kaair_mobile_bridge'

        # 사용할 맵 파일명 (파라미터로 오버라이드 가능)
        self.declare_parameter('map_filename', 'dure.stcm')
        self.map_filename = self.get_parameter('map_filename').get_parameter_value().string_value

        self.cli = self.create_client(
            SyncGetStcm,
            '/sync_get_stcm'
        )

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                'Waiting for sync_get_stcm service...'
            )

        self.req = SyncGetStcm.Request()

        try:
            package_share_directory = get_package_share_directory(
                package_name
            )

            maps_dir = os.path.join(
                package_share_directory,
                'config'
            )

            self.map_save_path = os.path.join(
                maps_dir,
                self.map_filename
            )

            # 파일 존재 여부 체크
            if not os.path.exists(self.map_save_path):
                self.get_logger().warn(
                    f"{self.map_filename} does not exist. "
                    f"A new file will be created."
                )

            self.get_logger().info(
                f"Using map path: {self.map_save_path}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to setup map path: {e}"
            )
            self.map_save_path = None

    def send_request(self):

        if not self.map_save_path:
            self.get_logger().error(
                "Map save path is not set."
            )
            return

        future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is not None:
            try:
                with open(self.map_save_path, "wb") as f:
                    f.write(future.result().raw_stcm)

                self.get_logger().info(
                    f"Map saved to {self.map_save_path}"
                )

            except IOError as e:
                self.get_logger().error(
                    f"Failed to write map: {e}"
                )

            except Exception as e:
                self.get_logger().error(
                    f"Unexpected error: {e}"
                )

        else:
            self.get_logger().error(
                "Failed to receive map data"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StcmMapSaver()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()