#!/usr/bin/env python3
"""
depth_resize_node.py

/femto/depth/image_raw 토픽을 구독하다가 입력 해상도가 정확히
576(h) x 640(w) 일 때만 720(h) x 1280(w) 로 리사이즈하여
/femto/depth/aligned 토픽으로 재발행한다.

- 해상도가 다르면 (카메라가 튀는 경우) drop 하고 throttle 경고 로그만 남긴다.
- header (stamp, frame_id) 는 원본 그대로 유지하여 TF/시간 동기화가 깨지지 않게 한다.
- depth 이미지이므로 보간법은 기본적으로 INTER_NEAREST 사용 (값 왜곡 방지).
  필요시 파라미터로 INTER_LINEAR 등으로 변경 가능.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DepthResizeNode(Node):
    def __init__(self):
        super().__init__('depth_resize_node')

        # ---- Parameters ----
        self.declare_parameter('input_topic', '/femto/depth/image_raw')
        self.declare_parameter('output_topic', '/femto/depth/aligned')
        self.declare_parameter('expected_input_height', 576)
        self.declare_parameter('expected_input_width', 640)
        self.declare_parameter('output_height', 720)
        self.declare_parameter('output_width', 1280)
        # nearest = depth 값 왜곡 없이 리사이즈 (권장)
        self.declare_parameter('interpolation', 'nearest')  # nearest | linear | area

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.expected_h = self.get_parameter('expected_input_height').value
        self.expected_w = self.get_parameter('expected_input_width').value
        self.out_h = self.get_parameter('output_height').value
        self.out_w = self.get_parameter('output_width').value

        interp_map = {
            'nearest': cv2.INTER_NEAREST,
            'linear': cv2.INTER_LINEAR,
            'area': cv2.INTER_AREA,
        }
        interp_param = self.get_parameter('interpolation').value
        self.interpolation = interp_map.get(interp_param, cv2.INTER_NEAREST)

        self.bridge = CvBridge()

        # 센서 데이터이므로 BEST_EFFORT QoS로 맞춤 (카메라 드라이버 기본값과 호환)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos)
        self.pub = self.create_publisher(Image, self.output_topic, qos)

        self._dropped_count = 0

        self.get_logger().info(
            f"[depth_resize_node] {self.input_topic} "
            f"(expect {self.expected_h}x{self.expected_w}) -> "
            f"{self.output_topic} ({self.out_h}x{self.out_w}), "
            f"interpolation={interp_param}"
        )

    def image_callback(self, msg: Image):
        in_h, in_w = msg.height, msg.width

        # 해상도가 예상과 다르면 드랍 (튀는 프레임 필터링)
        if in_h != self.expected_h or in_w != self.expected_w:
            self._dropped_count += 1
            self.get_logger().warn(
                f"Unexpected depth resolution {in_w}x{in_h} "
                f"(expected {self.expected_w}x{self.expected_h}) - dropping frame. "
                f"(total dropped: {self._dropped_count})",
                throttle_duration_sec=2.0
            )
            return

        try:
            # passthrough: 16UC1 / 32FC1 등 원본 인코딩 그대로 numpy로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        try:
            resized = cv2.resize(
                cv_image,
                (self.out_w, self.out_h),  # cv2.resize expects (width, height)
                interpolation=self.interpolation
            )
        except cv2.error as e:
            self.get_logger().error(f"cv2.resize failed: {e}")
            return

        try:
            out_msg = self.bridge.cv2_to_imgmsg(resized, encoding=msg.encoding)
        except CvBridgeError as e:
            self.get_logger().error(f"cv2_to_imgmsg conversion failed: {e}")
            return

        # 헤더(stamp, frame_id) 원본 그대로 유지 -> TF/시간 동기화 유지
        out_msg.header = msg.header

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthResizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()