


import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, CompressedImage, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as SciRot

from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros


@dataclass
class Config:
    input_depth: str = '/femto/depth/image_raw/compressedDepth'
    input_depth_info: str = '/femto/depth/camera_info'
    voxel_size: float = 0.05
    depth_scale: float = 1000.0   # mm → m
    depth_trunc: float = 5.0      # 최대 depth 거리 (m)
    planning_frame: str = 'base_link'   # MoveIt2 planning frame
    min_shape_confidence: float = 0.4   # 이 이하면 CollisionObject 생성 안 함


class SceneRepresentation(Node):
    def __init__(self):
        super().__init__('scene_representation_node')

        self.set_config()

        self.depth = None
        self.depth_header = None
        self.depth_intrinsic = None   # o3d.camera.PinholeCameraIntrinsic (camera_info 수신 후 설정)
        self.depth_frame_id   = ''     # camera_info header.frame_id

        # TF2: 카메라 프레임 → planning frame 변환용
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(f'SceneRepresentation started | {self.cfg}')

        # 카메라 드라이버 대부분이 BEST_EFFORT 사용
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_depth = self.create_subscription(
            CompressedImage,
            self.cfg.input_depth,
            self.callback_depth,
            qos_profile
        )
        self.sub_depth_info = self.create_subscription(
            CameraInfo,
            self.cfg.input_depth_info,
            self.callback_depth_info,
            qos_profile
        )

        self.pub_shape_markers = self.create_publisher(MarkerArray, '/scene/obstacle_markers', 10)
        self.pub_collision_obj = self.create_publisher(CollisionObject, '/collision_field', 10)

        self.timer = self.create_timer(0.1, self.callback_timer)

    def callback_timer(self):
        try:
            points = self._depth_to_voxel_pcd(self.depth)
            if points is None:
                return

            # self.pub_shape_markers.publish(points)
            # self.pub_collision_obj.publish(points)

        except Exception as e:
            self.get_logger().error(f'Error in callback_timer: {e}')


    def _depth_to_voxel_pcd(self, depth: np.ndarray) -> 'np.ndarray | None':
        """
        depth (원본 depth 해상도, uint16, mm 단위) →
          Open3D DepthImage → PointCloud (deproject)
          voxel_down_sample
          (N, 3) float32 반환
        """
        if self.depth_intrinsic is None:
            return None

        iw = self.depth_intrinsic.width
        ih = self.depth_intrinsic.height
        dh, dw = depth.shape

        # 해상도 불일치 시 intrinsics 해상도로 맞춤
        if dw != iw or dh != ih:
            depth_rs = cv2.resize(
                depth, (iw, ih), interpolation=cv2.INTER_NEAREST
            )
        else:
            depth_rs = depth

        depth_o3d = o3d.geometry.Image(depth_rs.astype(np.uint16))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d,
            self.depth_intrinsic,
            depth_scale=self.cfg.depth_scale,   # mm → m
            depth_trunc=self.cfg.depth_trunc,   # 최대 거리 (m)
        )

        pcd_ds = pcd.voxel_down_sample(self.cfg.voxel_size)
        pts = np.asarray(pcd_ds.points, dtype=np.float32)
        return pts if len(pts) > 0 else None

    # def callback_rgb(self, msg: CompressedImage):
    #     try:
    #         np_arr = np.frombuffer(msg.data, np.uint8).copy()  # frombuffer → read-only
    #         image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         self.scene_data.color = image
    #     except Exception as e:
    #         self.get_logger().error(f'Error in callback_rgb: {e}')
    #         return

    def callback_depth(self, msg: CompressedImage):
        """
        compressedDepth 포맷 처리:
          image_transport의 compressedDepth 플러그인은 일반 CompressedImage와 달리
          앞에 12바이트 헤더(depth quantization 파라미터)가 붙어 있음.
          → raw 버퍼에서 12바이트 skip 후 cv2.imdecode(IMREAD_UNCHANGED)로 uint16 획득.
        """
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8).copy()  # frombuffer → read-only

            # 12바이트 헤더 skip → PNG 영역만 디코딩
            depth = cv2.imdecode(raw[12:], cv2.IMREAD_UNCHANGED)

            # 헤더 길이가 다를 경우 fallback: 헤더 없이 전체 시도
            if depth is None:
                depth = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)

            if depth is None:
                self.get_logger().error('depth 디코딩 실패')
                return
            self.depth = depth
            self.depth_header = msg.header
            # self.get_logger().info(
            #     f'depth 수신: shape={depth.shape}, '
            #     f'dtype={depth.dtype}, '
            #     f'max={depth.max()}mm'
            # )

            # # 시각화용 (uint16 → uint8 변환, alpha=0.05 → 20000mm 기준)
            # depth_vis = cv2.convertScaleAbs(depth, alpha=0.05)
            # cv2.imshow('depth', depth_vis)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in callback_depth: {e}')

    def callback_depth_info(self, msg: CameraInfo):
        """depth 카메라 내부 파라미터 수신 (최초 1회만 설정)"""
        if self.depth_intrinsic is not None:
            return
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        w, h   = msg.width, msg.height
        self.depth_intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
        self.depth_frame_id   = msg.header.frame_id
        self.get_logger().info(
            f'Depth intrinsics 설정: {w}×{h}  fx={fx:.1f} fy={fy:.1f} '
            f'frame_id={self.depth_frame_id}'
        )

    def set_config(self):
        self.cfg = Config()

        self.declare_parameter('input_depth', '/femto/depth/image_raw/compressedDepth')
        self.declare_parameter('input_depth_info', '/femto/depth/camera_info')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('depth_trunc', 5.0)
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('min_shape_confidence', 0.4)

        self.cfg.input_depth           = self.get_parameter('input_depth').value
        self.cfg.input_depth_info      = self.get_parameter('input_depth_info').value
        self.cfg.voxel_size            = self.get_parameter('voxel_size').value
        self.cfg.depth_scale           = self.get_parameter('depth_scale').value
        self.cfg.depth_trunc           = self.get_parameter('depth_trunc').value
        self.cfg.planning_frame        = self.get_parameter('planning_frame').value
        self.cfg.min_shape_confidence  = self.get_parameter('min_shape_confidence').value


def main(args=None):
    rclpy.init(args=args)
    node = SceneRepresentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
