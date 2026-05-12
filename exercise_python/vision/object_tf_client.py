#!/usr/bin/env python3

import sys
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image

from pyconnect.utils import init_detect_client
from kaair_msgs.srv import CreateObjectMarker


class OneShotCreateObjectTFClient(Node):
    def __init__(self, object_label: str):
        super().__init__('one_shot_create_object_tf_client')

        self.object_label = object_label.strip() if object_label.strip() else 'object'
        self.latest_rgb_msg = None
        self.rgb_event = threading.Event()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/femto/color/image_raw',
            self.rgb_callback,
            sensor_qos
        )

        self.cli = self.create_client(
            CreateObjectMarker,
            '/create_object_marker'
        )

        self.detect_client = init_detect_client(
            detect_url='tcp:192.168.1.18:8805'
        )

    def rgb_callback(self, msg):
        self.latest_rgb_msg = msg
        self.rgb_event.set()

    def ros_image_to_numpy(self, msg):
        if msg.encoding not in ['rgb8', 'bgr8']:
            self.get_logger().error(f'Unsupported RGB encoding: {msg.encoding}')
            return None

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )

        if msg.encoding == 'bgr8':
            img = img[:, :, ::-1]

        return img

    def detect_object_center(self):
        if self.latest_rgb_msg is None:
            self.get_logger().error('No RGB image received yet.')
            return None

        rgb_np = self.ros_image_to_numpy(self.latest_rgb_msg)
        if rgb_np is None:
            return None

        try:
            ins = self.detect_client.send({
                'rgb': rgb_np,
                'caption': self.object_label,
                'detector': 'groundingdino'
            })
        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
            return None

        if self.object_label not in ins.label_clusters:
            self.get_logger().warn(
                f'Object "{self.object_label}" not found in label_clusters.'
            )
            return None

        centers = ins.label_clusters[self.object_label].centers
        if centers is None or len(centers) == 0:
            self.get_logger().warn(f'No centers found for "{self.object_label}".')
            return None

        u = int(centers[0][0])
        v = int(centers[0][1])
        return (u, v)

    def wait_for_rgb_once(self, timeout_sec=3.0):
        self.get_logger().info('Waiting for one RGB frame...')
        return self.rgb_event.wait(timeout=timeout_sec)

    def wait_for_service_ready(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, object_label, u, v):
        req = CreateObjectMarker.Request()
        req.object_label = object_label
        req.u = int(u)
        req.v = int(v)
        return self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    object_label = 'object'
    if len(sys.argv) > 1:
        object_label = sys.argv[1]

    node = OneShotCreateObjectTFClient(object_label)

    try:
        # RGB 1장 받을 때까지 spin
        while rclpy.ok() and not node.rgb_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.latest_rgb_msg is None:
            node.get_logger().error('Failed to receive RGB image.')
            return

        center = node.detect_object_center()
        if center is None:
            node.get_logger().error('Failed to detect object center.')
            return

        u, v = center
        node.get_logger().info(
            f'Detected "{object_label}" center at pixel ({u}, {v})'
        )

        node.wait_for_service_ready()
        future = node.send_request(object_label, u, v)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            node.get_logger().info(
                f'Result: success={response.success}, message="{response.message}"'
            )
        else:
            node.get_logger().error('Service call failed.')

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()