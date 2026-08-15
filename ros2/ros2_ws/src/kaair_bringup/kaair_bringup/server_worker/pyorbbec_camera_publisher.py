#!/usr/bin/env python3

"""
pyorbbec_camera_publisher

pyorbbecsdk(OrbbecSDK 공식 Python 바인딩, pip install pyorbbecsdk)를 직접
사용해 Femto 시리즈 카메라의 color/depth 스트림을 열고, ROS2
sensor_msgs/Image(color, depth) + sensor_msgs/CameraInfo(color, depth)로
발행한다.

orbbec_camera(OrbbecSDK_ROS2) C++ 드라이버 launch를 그대로 쓰지 않고 SDK를
직접 제어해야 하는 상황(파이프라인 커스터마이징, 별도 프로세스 분리 등)을
위한 대체/보조 노드. 기본 launcher(server_worker_loader.py)에는 등록하지
않으며, 필요할 때 `ros2 run kaair_bringup pyorbbec_camera_publisher`로 직접
실행한다.

[정렬]
  depth는 AlignFilter(align_to_stream=COLOR_STREAM)로 color 프레임의 해상도/
  시점에 맞춰 정렬한 뒤 발행한다. 정렬된 depth는 color 카메라 모델을 그대로
  따르므로, depth CameraInfo도 color 스트림의 intrinsics를 사용한다.

[타임스탬프]
  프레임의 SDK system timestamp(호스트가 프레임을 수신한 시각, us 단위)를
  ROS Time으로 변환해 사용한다. 디바이스 하드웨어 clock과 ROS(호스트) clock을
  별도로 동기화할 필요 없이 항상 host wall-clock 기준의 유효한 타임스탬프가
  나온다. system timestamp를 가져올 수 없는 SDK 버전에서는 노드의 clock으로
  대체한다.

[frame_id]
  color_frame_id / depth_frame_id 파라미터로 지정한다 (기본값:
  femto_color_optical_frame / femto_depth_optical_frame).
"""

import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

from pyorbbecsdk import (
    AlignFilter,
    Config,
    Context,
    OBFormat,
    OBSensorType,
    OBStreamType,
    Pipeline,
)

# color_format 파라미터 문자열 -> (OBFormat, sensor_msgs Image encoding)
_COLOR_FORMATS = {
    "MJPG": (OBFormat.MJPG, "bgr8"),  # cv2.imdecode로 디코딩 -> BGR
    "RGB": (OBFormat.RGB, "rgb8"),
}


class PyorbbecCameraPublisher(Node):
    def __init__(self):
        super().__init__("pyorbbec_camera_publisher")

        self.declare_parameter("serial_number", "")

        self.declare_parameter("color_width", 1280)
        self.declare_parameter("color_height", 720)
        self.declare_parameter("color_fps", 30)
        self.declare_parameter("color_format", "MJPG")

        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 576)
        self.declare_parameter("depth_fps", 30)

        self.declare_parameter("color_topic", "/femto/color/image_raw")
        self.declare_parameter("color_info_topic", "/femto/color/camera_info")
        self.declare_parameter("depth_topic", "/femto/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/femto/depth/camera_info")

        self.declare_parameter("color_frame_id", "femto_color_optical_frame")
        self.declare_parameter("depth_frame_id", "femto_depth_optical_frame")

        self.declare_parameter("wait_frames_timeout_ms", 100)

        self.serial_number = self.get_parameter("serial_number").value

        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.color_fps = int(self.get_parameter("color_fps").value)
        color_format_name = str(self.get_parameter("color_format").value).upper()
        if color_format_name not in _COLOR_FORMATS:
            self.get_logger().warn(
                f"지원하지 않는 color_format '{color_format_name}', MJPG로 대체"
            )
            color_format_name = "MJPG"
        self.color_format_name = color_format_name
        self.ob_color_format, self.color_encoding = _COLOR_FORMATS[color_format_name]

        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fps = int(self.get_parameter("depth_fps").value)

        self.color_topic = self.get_parameter("color_topic").value
        self.color_info_topic = self.get_parameter("color_info_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.depth_info_topic = self.get_parameter("depth_info_topic").value

        self.color_frame_id = self.get_parameter("color_frame_id").value
        self.depth_frame_id = self.get_parameter("depth_frame_id").value

        self.wait_frames_timeout_ms = int(self.get_parameter("wait_frames_timeout_ms").value)

        if self.color_format_name == "MJPG" and not _HAS_CV2:
            raise RuntimeError(
                "color_format=MJPG 에는 cv2(OpenCV Python 바인딩)가 필요합니다. "
                "cv2가 없으면 color_format을 'RGB'로 바꾸세요."
            )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.color_pub = self.create_publisher(Image, self.color_topic, sensor_qos)
        self.color_info_pub = self.create_publisher(CameraInfo, self.color_info_topic, sensor_qos)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, sensor_qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.depth_info_topic, sensor_qos)

        self._align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self._pipeline = None
        self._stop_event = threading.Event()
        self.log_flags = {}

        self._start_pipeline()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.get_logger().info(
            f"pyorbbec_camera_publisher: color[{self.color_width}x{self.color_height}@"
            f"{self.color_fps}fps,{self.color_format_name}] -> {self.color_topic} "
            f"(frame={self.color_frame_id}) | "
            f"depth[{self.depth_width}x{self.depth_height}@{self.depth_fps}fps -> "
            f"color 해상도로 정렬] -> {self.depth_topic} (frame={self.depth_frame_id})"
        )

    # ──────────────────────────────────────────────
    # Logging helper
    # ──────────────────────────────────────────────

    def _log_once(self, key, message, level="info"):
        if self.log_flags.get(key, False):
            return
        getattr(self.get_logger(), level)(message)
        self.log_flags[key] = True

    # ──────────────────────────────────────────────
    # Pipeline setup
    # ──────────────────────────────────────────────

    def _open_device(self):
        if not self.serial_number:
            return None  # Pipeline()이 기본 장치를 연다

        ctx = Context()
        device_list = ctx.query_devices()
        device = device_list.get_device_by_serial_number(self.serial_number)
        if device is None:
            raise RuntimeError(
                f"시리얼 번호 '{self.serial_number}'에 해당하는 Orbbec 장치를 찾을 수 없음"
            )
        return device

    def _start_pipeline(self):
        device = self._open_device()
        pipeline = Pipeline(device) if device is not None else Pipeline()
        config = Config()

        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile = color_profiles.get_video_stream_profile(
                self.color_width, self.color_height, self.ob_color_format, self.color_fps
            )
        except Exception as e:
            self.get_logger().warn(
                f"요청한 color 프로파일을 찾지 못해 기본 프로파일 사용: {e}"
            )
            color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        try:
            depth_profile = depth_profiles.get_video_stream_profile(
                self.depth_width, self.depth_height, OBFormat.Y16, self.depth_fps
            )
        except Exception as e:
            self.get_logger().warn(
                f"요청한 depth 프로파일을 찾지 못해 기본 프로파일 사용: {e}"
            )
            depth_profile = depth_profiles.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        pipeline.enable_frame_sync()
        pipeline.start(config)

        self._pipeline = pipeline
        self.get_logger().info("Orbbec pipeline 시작됨")

    # ──────────────────────────────────────────────
    # Capture loop (별도 스레드에서 실행: wait_for_frames가 blocking 호출)
    # ──────────────────────────────────────────────

    def _capture_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                frames = self._pipeline.wait_for_frames(self.wait_frames_timeout_ms)
            except Exception as e:
                self._log_once("wait_frames_error", f"wait_for_frames 실패: {e}", level="warn")
                continue
            if frames is None:
                continue

            aligned = self._align_filter.process(frames)
            if aligned is None:
                continue
            aligned = aligned.as_frame_set()

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if color_frame is None or depth_frame is None:
                continue

            stamp = self._frame_stamp(color_frame)

            try:
                self._publish_color(color_frame, stamp)
                self._publish_depth(depth_frame, color_frame, stamp)
            except Exception as e:
                self._log_once("publish_error", f"프레임 발행 실패: {e}", level="error")

    # ──────────────────────────────────────────────
    # Timestamp
    # ──────────────────────────────────────────────

    def _frame_stamp(self, frame):
        """SDK system timestamp(us, 호스트 도착 시각) -> ROS Time.

        디바이스 하드웨어 clock이 아닌 호스트가 프레임을 수신한 시각이므로
        ROS(호스트) clock과 같은 시간 축을 공유해 TF 등에서 바로 사용 가능.
        """
        ts_us = None
        getter = getattr(frame, "get_system_timestamp_us", None)
        if callable(getter):
            try:
                ts_us = float(getter())
            except Exception:
                ts_us = None

        if ts_us is None:
            return self.get_clock().now().to_msg()

        return Time(nanoseconds=int(ts_us * 1000.0)).to_msg()

    # ──────────────────────────────────────────────
    # Color
    # ──────────────────────────────────────────────

    def _publish_color(self, frame, stamp):
        width = frame.get_width()
        height = frame.get_height()
        raw = np.asanyarray(frame.get_data())

        if self.color_format_name == "MJPG":
            img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if img is None:
                self._log_once("mjpg_decode_fail", "MJPG 디코딩 실패, 프레임 스킵", level="warn")
                return
            height, width = img.shape[:2]
            step = width * 3
        else:  # RGB
            img = raw.reshape((height, width, 3))
            step = width * 3

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.color_frame_id
        msg.height = height
        msg.width = width
        msg.encoding = self.color_encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = np.ascontiguousarray(img).tobytes()
        self.color_pub.publish(msg)

        info = self._camera_info_from_profile(
            frame.get_stream_profile().as_video_stream_profile(),
            self.color_frame_id, stamp, width, height,
        )
        self.color_info_pub.publish(info)

        self._log_once(
            "color_published",
            f"First color frame published: {width}x{height} ({self.color_encoding}) "
            f"-> {self.color_topic}",
        )

    # ──────────────────────────────────────────────
    # Depth (color 해상도로 정렬된 상태)
    # ──────────────────────────────────────────────

    def _publish_depth(self, depth_frame, color_frame, stamp):
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()

        raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
        depth_mm = (raw.astype(np.float32) * float(scale)).astype(np.uint16)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.depth_frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = width * 2
        msg.data = depth_mm.tobytes()
        self.depth_pub.publish(msg)

        # AlignFilter로 depth를 color 시점/해상도에 맞췄으므로, depth의
        # CameraInfo도 color 스트림의 intrinsics를 그대로 사용한다.
        info = self._camera_info_from_profile(
            color_frame.get_stream_profile().as_video_stream_profile(),
            self.depth_frame_id, stamp, width, height,
        )
        self.depth_info_pub.publish(info)

        self._log_once(
            "depth_published",
            f"First depth frame published: {width}x{height} (16UC1, color 해상도로 정렬) "
            f"-> {self.depth_topic}",
        )

    # ──────────────────────────────────────────────
    # CameraInfo
    # ──────────────────────────────────────────────

    def _camera_info_from_profile(self, video_profile, frame_id, stamp, width, height):
        intr = video_profile.get_intrinsic()
        dist = video_profile.get_distortion()

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [
            float(dist.k1), float(dist.k2), float(dist.p1), float(dist.p2), float(dist.k3)
        ]
        info.k = [
            float(intr.fx), 0.0, float(intr.cx),
            0.0, float(intr.fy), float(intr.cy),
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            float(intr.fx), 0.0, float(intr.cx), 0.0,
            0.0, float(intr.fy), float(intr.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    # ──────────────────────────────────────────────
    # Shutdown
    # ──────────────────────────────────────────────

    def destroy_node(self):
        self._stop_event.set()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as e:
                self.get_logger().warn(f"pipeline.stop() 실패: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PyorbbecCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
