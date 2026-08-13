#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def transform_to_pose_stamped(tf_msg) -> PoseStamped:
    out = PoseStamped()
    out.header = tf_msg.header
    out.pose.position.x = tf_msg.transform.translation.x
    out.pose.position.y = tf_msg.transform.translation.y
    out.pose.position.z = tf_msg.transform.translation.z
    out.pose.orientation = tf_msg.transform.rotation
    return out


class RobotPosePublisher(Node):
    """Publishes poses from /tf as PoseStamped (x, y, z + quaternion)."""

    def __init__(self):
        super().__init__('robot_pose_publisher')

        self.declare_parameter('map_frame', 'slamware_map')
        self.declare_parameter('mobile_base_frame', 'base_footprint')
        self.declare_parameter('arm_base_frame', 'arm_base')
        self.declare_parameter('tool_frame', 'tool_tcp_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('tf_lookup_timeout', 0.2)
        self.declare_parameter('publish_map_tool_tcp', True)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.mobile_base_frame = (
            self.get_parameter('mobile_base_frame').get_parameter_value().string_value
        )
        self.arm_base_frame = self.get_parameter('arm_base_frame').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
        rate = float(self.get_parameter('publish_rate').value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_lookup_timeout').value))
        self.publish_map_tool_tcp = bool(self.get_parameter('publish_map_tool_tcp').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_map_base = self.create_publisher(
            PoseStamped, '/robot_pose/mobile_pose', 10
        )
        self.pub_arm_tool = self.create_publisher(
            PoseStamped, '/robot_pose/arm_base_tool_pose', 10
        )
        self.pub_map_tool = self.create_publisher(
            PoseStamped, '/robot_pose/map_base_tool_pose', 10
        )
        self.pub_base_tool = self.create_publisher(
            PoseStamped, '/robot_pose/mobile_base_tool_pose', 10
        )

        period = 1.0 / rate if rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'TF pose publisher: {self.map_frame}<-{self.mobile_base_frame}, '
            f'{self.arm_base_frame}<-{self.tool_frame}, '
            f'{self.mobile_base_frame}<-{self.tool_frame}'
            + (f', {self.map_frame}<-{self.tool_frame}' if self.publish_map_tool_tcp else '')
        )

    def _try_lookup(self, target_frame: str, source_frame: str):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=self.tf_timeout,
        )

    def on_timer(self):
        try:
            tf_mb = self._try_lookup(self.map_frame, self.mobile_base_frame)
            self.pub_map_base.publish(transform_to_pose_stamped(tf_mb))
        except Exception as ex:
            self.get_logger().debug(
                f'TF {self.map_frame} <- {self.mobile_base_frame}: {ex}'
            )

        try:
            tf_at = self._try_lookup(self.arm_base_frame, self.tool_frame)
            self.pub_arm_tool.publish(transform_to_pose_stamped(tf_at))
        except Exception as ex:
            self.get_logger().debug(
                f'TF {self.arm_base_frame} <- {self.tool_frame}: {ex}'
            )

        try:
            tf_bt = self._try_lookup(self.mobile_base_frame, self.tool_frame)
            self.pub_base_tool.publish(transform_to_pose_stamped(tf_bt))
        except Exception as ex:
            self.get_logger().debug(
                f'TF {self.mobile_base_frame} <- {self.tool_frame}: {ex}'
            )

        if self.publish_map_tool_tcp:
            try:
                tf_mt = self._try_lookup(self.map_frame, self.tool_frame)
                self.pub_map_tool.publish(transform_to_pose_stamped(tf_mt))
            except Exception as ex:
                self.get_logger().debug(
                    f'TF {self.map_frame} <- {self.tool_frame}: {ex}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
