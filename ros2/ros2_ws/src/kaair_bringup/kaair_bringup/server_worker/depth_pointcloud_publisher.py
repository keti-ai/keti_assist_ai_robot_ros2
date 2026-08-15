#!/usr/bin/env python3

"""
depth_pointcloud_publisher

/femto/depth/aligned (depth image) + /femto/depth/camera_info 를 구독해
프레임마다 픽셀 → 3D 포인트를 벡터화(numpy)로 계산하고, TF를 이용해
map(고정) 프레임으로 변환한 뒤 RViz2에서 PointCloud2 디스플레이로 바로
구독 가능한 토픽(기본 /femto/marker)으로 실시간 발행한다.

object_marker_server.py 의 프레임/좌표 변환 방식(카메라 intrinsics로 픽셀→3D
투영 후 TF로 map까지 변환)과 동일한 원리이지만, 서비스 요청으로 들어온 몇 개의
(u, v) 픽셀만 처리하는 대신 전체 depth 프레임을 numpy로 한 번에 처리해
실시간 스트리밍이 가능하도록 만들어졌다.

중요: Orbbec 드라이버는 CPU 부하 등으로 depth_registration 전/후 해상도의
camera_info 를 번갈아 publish 할 수 있다. aligned depth는 1280x720으로
고정인데 native 해상도 intrinsics(fx/fy/cx/cy)가 섞이면 포인트클라우드가
통째로 밀리거나 기울어 보인다. 그래서 expected 해상도와 일치하는
camera_info 만 채택한다.
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

SUPPORTED_ENCODINGS = ("16UC1", "mono16", "32FC1")


class DepthPointCloudPublisher(Node):
    def __init__(self):
        super().__init__("depth_pointcloud_publisher")

        self.declare_parameter("depth_topic", "/femto/depth/aligned")
        self.declare_parameter("camera_info_topic", "/femto/depth/camera_info")
        self.declare_parameter("output_topic", "/femto/marker")
        self.declare_parameter("map_frame", "slamware_map")
        # depth_resizer / object_marker_server 와 동일한 aligned 해상도.
        # camera_info.width/height 가 이 값과 다를 때는 무시한다.
        self.declare_parameter("expected_width", 1280)
        self.declare_parameter("expected_height", 720)
        # 픽셀을 stride 간격으로 건너뛰며 샘플링 (해상도 1280x720에서
        # stride=4 → 약 320x180 = 57,600 포인트/프레임)
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.expected_width = int(self.get_parameter("expected_width").value)
        self.expected_height = int(self.get_parameter("expected_height").value)
        self.stride = max(1, int(self.get_parameter("stride").value))
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # expected 해상도와 일치하는 camera_info 만 보관한다.
        self.latest_camera_info = None
        self.log_flags = {}

        # 픽셀 그리드(u,v)는 해상도/stride가 바뀌지 않는 한 동일하므로 캐시한다.
        self._grid_key = None
        self._u_idx = None
        self._v_idx = None
        self._uu = None
        self._vv = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            sensor_qos,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            sensor_qos,
        )

        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)

        self.get_logger().info(
            f"depth_pointcloud_publisher: {self.depth_topic} + {self.camera_info_topic} "
            f"-> {self.output_topic} (frame={self.map_frame}, "
            f"expect={self.expected_width}x{self.expected_height}, "
            f"stride={self.stride}, "
            f"range=[{self.min_range:.2f},{self.max_range:.2f}]m)"
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
    # Subscriptions
    # ──────────────────────────────────────────────

    def _is_expected_size(self, width, height):
        return (
            int(width) == self.expected_width and
            int(height) == self.expected_height
        )

    def _camera_info_callback(self, msg: CameraInfo):
        # 드라이버가 native depth 해상도 camera_info 를 섞어 보내는 경우가 있다.
        # aligned depth(1280x720)와 짝이 맞지 않는 intrinsics 는 채택하지 않는다.
        # 이미 유효한 camera_info 가 있으면 그것을 유지한다.
        if not self._is_expected_size(msg.width, msg.height):
            suffix = (
                "keeping previous valid intrinsics"
                if self.latest_camera_info is not None
                else "waiting for matching camera_info"
            )
            self.get_logger().warn(
                f"Ignoring camera_info with unexpected resolution "
                f"{msg.width}x{msg.height} "
                f"(expected {self.expected_width}x{self.expected_height}); {suffix}",
                throttle_duration_sec=2.0,
            )
            return

        prev = self.latest_camera_info
        self.latest_camera_info = msg

        if prev is None:
            self.get_logger().info(
                f"Accepted camera_info: {msg.width}x{msg.height}, "
                f"fx={msg.k[0]:.3f}, fy={msg.k[4]:.3f}, "
                f"cx={msg.k[2]:.3f}, cy={msg.k[5]:.3f}, "
                f"frame_id={msg.header.frame_id}"
            )
        elif (
            abs(prev.k[0] - msg.k[0]) > 1e-3 or
            abs(prev.k[4] - msg.k[4]) > 1e-3 or
            abs(prev.k[2] - msg.k[2]) > 1e-3 or
            abs(prev.k[5] - msg.k[5]) > 1e-3
        ):
            # 같은 해상도인데 intrinsics 가 바뀌면 shifting 원인이 될 수 있어 로그
            self.get_logger().warn(
                f"camera_info intrinsics changed at {msg.width}x{msg.height}: "
                f"fx {prev.k[0]:.3f}->{msg.k[0]:.3f}, "
                f"fy {prev.k[4]:.3f}->{msg.k[4]:.3f}, "
                f"cx {prev.k[2]:.3f}->{msg.k[2]:.3f}, "
                f"cy {prev.k[5]:.3f}->{msg.k[5]:.3f}",
                throttle_duration_sec=2.0,
            )

    def _depth_callback(self, msg: Image):
        if self.latest_camera_info is None:
            self._log_once(
                "no_camera_info",
                "유효한(해상도 일치) camera_info를 아직 받지 못해 "
                "point cloud를 생성할 수 없음.",
                level="warn",
            )
            return

        # depth_resizer 가 이미 필터하지만, 방어적으로 한 번 더 검사
        if not self._is_expected_size(msg.width, msg.height):
            self.get_logger().warn(
                f"Unexpected depth resolution {msg.width}x{msg.height} "
                f"(expected {self.expected_width}x{self.expected_height}) - dropping",
                throttle_duration_sec=2.0,
            )
            return

        cam = self.latest_camera_info
        if cam.width != msg.width or cam.height != msg.height:
            # 이론상 _camera_info_callback 에서 걸러지지만 안전장치
            self.get_logger().warn(
                f"camera_info ({cam.width}x{cam.height})와 depth "
                f"({msg.width}x{msg.height}) 해상도 불일치 - dropping",
                throttle_duration_sec=2.0,
            )
            return

        z = self._decode_depth_meters(msg)
        if z is None:
            return

        points_cam = self._project_to_points(z, cam)
        if points_cam.shape[0] == 0:
            return

        points_target = self._transform_points(
            points_cam, msg.header.frame_id, self.map_frame
        )
        if points_target is None:
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.map_frame
        cloud_msg = pc2.create_cloud_xyz32(header, points_target.astype(np.float32))
        self.cloud_pub.publish(cloud_msg)

        self._log_once(
            "cloud_published",
            f"First point cloud published: {points_target.shape[0]} points, "
            f"frame={self.map_frame}, topic={self.output_topic}",
        )

    # ──────────────────────────────────────────────
    # Depth decoding
    # ──────────────────────────────────────────────

    def _decode_depth_meters(self, msg: Image):
        """depth Image 메시지를 (height, width) 크기의 float32 미터 배열로 변환한다.

        무효 픽셀(0 또는 비유한 값)은 NaN으로 표시한다. step에 패딩이 있는
        경우까지 고려해 행마다 실제 유효 바이트만 잘라서 사용한다.
        """
        encoding = msg.encoding
        if encoding not in SUPPORTED_ENCODINGS:
            self._log_once(
                f"unsupported_encoding_{encoding}",
                f"지원하지 않는 depth 인코딩: {encoding} "
                f"({'/'.join(SUPPORTED_ENCODINGS)}만 지원)",
                level="warn",
            )
            return None

        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        except ValueError as e:
            self.get_logger().error(f"depth 이미지 버퍼 reshape 실패: {e}")
            return None

        if encoding in ("16UC1", "mono16"):
            arr = raw[:, : msg.width * 2].view(np.uint16).reshape(msg.height, msg.width)
            z = arr.astype(np.float32) / 1000.0
            z[arr == 0] = np.nan
        else:  # 32FC1
            arr = raw[:, : msg.width * 4].view(np.float32).reshape(msg.height, msg.width)
            z = arr.astype(np.float32)
            z[~np.isfinite(z) | (z <= 0.0)] = np.nan

        return z

    # ──────────────────────────────────────────────
    # Pixel → 3D projection (vectorized)
    # ──────────────────────────────────────────────

    def _ensure_grid(self, height, width):
        key = (height, width, self.stride)
        if self._grid_key == key:
            return
        self._v_idx = np.arange(0, height, self.stride)
        self._u_idx = np.arange(0, width, self.stride)
        self._uu, self._vv = np.meshgrid(self._u_idx, self._v_idx)
        self._grid_key = key
        self.get_logger().info(
            f"Pixel grid (재)생성: {width}x{height}, stride={self.stride} "
            f"-> {self._uu.size} samples/frame"
        )

    def _project_to_points(self, z, cam: CameraInfo):
        height, width = z.shape
        self._ensure_grid(height, width)

        fx, fy = float(cam.k[0]), float(cam.k[4])
        cx, cy = float(cam.k[2]), float(cam.k[5])
        if fx <= 1e-9 or fy <= 1e-9:
            self.get_logger().error(
                f"Invalid camera intrinsics fx={fx}, fy={fy}",
                throttle_duration_sec=2.0,
            )
            return np.empty((0, 3), dtype=np.float32)

        z_sub = z[np.ix_(self._v_idx, self._u_idx)]
        valid = np.isfinite(z_sub) & (z_sub > self.min_range) & (z_sub < self.max_range)

        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        zz = z_sub[valid]
        xx = (self._uu[valid].astype(np.float32) - cx) * zz / fx
        yy = (self._vv[valid].astype(np.float32) - cy) * zz / fy

        return np.stack([xx, yy, zz], axis=-1)

    # ──────────────────────────────────────────────
    # TF transform (vectorized: 프레임당 한 번만 조회 후 전체 포인트에 적용)
    # ──────────────────────────────────────────────

    def _transform_points(self, points_cam, source_frame, target_frame):
        if source_frame == target_frame:
            return points_cam

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as ex:
            self._log_once(
                f"tf_fail_{source_frame}",
                f"TF lookup 실패 {target_frame} <- {source_frame}: {ex}",
                level="warn",
            )
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        rot_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        return points_cam.astype(np.float64) @ rot_matrix.T + translation


def main(args=None):
    rclpy.init(args=args)
    node = DepthPointCloudPublisher()
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
