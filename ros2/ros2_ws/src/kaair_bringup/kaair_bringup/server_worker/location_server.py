#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from kaair_msgs.msg import PlaceInfo


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class PlaceInfoNode(Node):
    def __init__(self):
        super().__init__('place_info_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('mobile_height', 280.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('frame_id', 'slamware_map')

        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.mobile_height = float(self.get_parameter('mobile_height').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value

        if self.config_file == '':
            pkg_share = get_package_share_directory('kaair_bringup')
            self.config_file = os.path.join(pkg_share, 'config', 'maps', 'kcare_lab.yaml')
        self.config_file = os.path.expanduser(self.config_file)
        if not os.path.isfile(self.config_file):
            raise FileNotFoundError(f'Place config file not found: {self.config_file}')

        self.place_data = self.load_yaml(self.config_file)
        self.last_mtime = os.path.getmtime(self.config_file)

        self.info_pub = self.create_publisher(PlaceInfo, '/place_info', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/place_markers', 10)

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_all)
        self.reload_timer = self.create_timer(2.0, self.check_reload)

        self.get_logger().info(f'Loaded config: {self.config_file}')
        self.get_logger().info(f'Using marker frame: {self.frame_id}')

    def load_yaml(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data if data is not None else {}

    def check_reload(self):
        try:
            current_mtime = os.path.getmtime(self.config_file)
            if current_mtime != self.last_mtime:
                self.place_data = self.load_yaml(self.config_file)
                self.last_mtime = current_mtime
                self.get_logger().info('YAML file changed. Reloaded.')
        except Exception as e:
            self.get_logger().error(f'Failed to reload yaml: {e}')

    def build_place_msg(self, room_name, place_name, place_dict):
        msg = PlaceInfo()
        msg.room = str(room_name)
        msg.place = str(place_name)
        msg.handle = str(place_dict.get('handle', ''))
        msg.default_mode = str(place_dict.get('default_mode', ''))

        placepose = place_dict.get('placepose', {})
        msg.place_dx = float(placepose.get('dx', 0.0))
        msg.place_dy = float(placepose.get('dy', 0.0))

        raw_height = float(place_dict.get('height', 0.0))
        msg.height = raw_height - self.mobile_height

        loc = place_dict.get('loc', {})
        msg.loc_x = float(loc.get('x', 0.0))
        msg.loc_y = float(loc.get('y', 0.0))
        msg.loc_theta = float(loc.get('theta', 0.0))

        msg.dforward = float(place_dict.get('dforward', 0.0))
        msg.dshift = float(place_dict.get('dshift', 0.0))

        return msg

    def build_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        now = self.get_clock().now().to_msg()

        for room_name, room_dict in self.place_data.items():
            if not isinstance(room_dict, dict):
                continue

            for place_name, place_dict in room_dict.items():
                if place_name == 'default_loc':
                    continue
                if not isinstance(place_dict, dict):
                    continue

                loc = place_dict.get('loc', {})
                x = float(loc.get('x', 0.0)) / 1000.0
                y = float(loc.get('y', 0.0)) / 1000.0
                theta_deg = float(loc.get('theta', 0.0))
                theta_rad = math.radians(theta_deg)

                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = now
                arrow.ns = 'place_arrow'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = x
                arrow.pose.position.y = y
                arrow.pose.position.z = 0.0
                arrow.pose.orientation = yaw_to_quaternion(theta_rad)
                arrow.scale.x = 0.4
                arrow.scale.y = 0.08
                arrow.scale.z = 0.08
                arrow.color.a = 1.0
                arrow.color.r = 0.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0
                marker_array.markers.append(arrow)

                text = Marker()
                text.header.frame_id = self.frame_id
                text.header.stamp = now
                text.ns = 'place_text'
                text.id = marker_id
                marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = 0.3
                text.pose.orientation.w = 1.0
                text.scale.z = 0.2
                text.color.a = 1.0
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.text = f'{room_name}/{place_name}'
                marker_array.markers.append(text)

        return marker_array

    def publish_all(self):
        for room_name, room_dict in self.place_data.items():
            if not isinstance(room_dict, dict):
                continue

            for place_name, place_dict in room_dict.items():
                if place_name == 'default_loc':
                    continue
                if not isinstance(place_dict, dict):
                    continue

                msg = self.build_place_msg(room_name, place_name, place_dict)
                self.info_pub.publish(msg)

        marker_array = self.build_markers()
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PlaceInfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
