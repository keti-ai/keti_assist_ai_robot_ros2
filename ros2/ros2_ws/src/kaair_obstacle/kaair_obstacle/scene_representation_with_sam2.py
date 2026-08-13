


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

from kaair_obstacle.utils.sam_wrapper import Sam2MaskWrapper
from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros
from kaair_obstacle.utils.shape_estimator import estimate_shape, ShapeResult, ShapeType


@dataclass
class Config:
    input_rgb: str = '/femto/color/image_raw/compressed'
    input_depth: str = '/femto/depth/image_raw/compressedDepth'
    input_depth_info: str = '/femto/depth/camera_info'
    input_pcd: str = '/femto/depth_registered/points'
    sam2_model: str = 'sam2.1_hiera_large.pt'
    sam2_cfg: str = 'sam2.1_hiera_l.yaml'
    voxel_size: float = 0.05
    depth_scale: float = 1000.0   # mm → m
    depth_trunc: float = 5.0      # 최대 depth 거리 (m)
    planning_frame: str = 'base_link'   # MoveIt2 planning frame
    min_shape_confidence: float = 0.4   # 이 이하면 CollisionObject 생성 안 함


@dataclass
class SceneData:
    color = None
    depth = None
    color_align_depth = None  # color, depth=0 픽셀 마스킹된 color
    depth_aligned = None      # color 해상도로 resize된 depth (uint16)
    masks = None              # SAM2 마스크 리스트 (각 mask에 'depth' 키 추가됨)
    depth_without_mask = None # 어떤 마스크에도 속하지 않는 depth 픽셀 (원본 해상도)
    pcd_without_mask = None   # depth_without_mask → voxel 다운샘플된 포인트 (N,3) float32
    result_img = None
    pcd = None
    depth_header = None       # 마지막 수신 depth 메시지 헤더
    shape_results = None      # List[ShapeResult] — 마스크별 형태 추정 결과

class SceneRepresentation(Node):
    def __init__(self):
        super().__init__('scene_representation_node')

        self.set_config()

        self.scene_data = SceneData()
        self._depth_intrinsics = None   # o3d.camera.PinholeCameraIntrinsic (camera_info 수신 후 설정)
        self._depth_frame_id   = ''     # camera_info header.frame_id

        # TF2: 카메라 프레임 → planning frame 변환용
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(f'SceneRepresentation started | {self.cfg}')
        self.sam_wrapper = Sam2MaskWrapper(self)

        # 카메라 드라이버 대부분이 BEST_EFFORT 사용
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_rgb = self.create_subscription(
            CompressedImage,
            self.cfg.input_rgb,
            self.callback_rgb,
            qos_profile
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

        self.pub_background_cloud = self.create_publisher(
            PointCloud2, '/scene/background_cloud', 10
        )
        self.pub_shape_markers = self.create_publisher(
            MarkerArray, '/scene/shape_markers', 10
        )
        # MoveIt2 PlanningScene에 CollisionObject를 직접 적용하는 토픽
        self.pub_planning_scene = self.create_publisher(
            PlanningScene, '/monitored_planning_scene', 10
        )

        self.timer = self.create_timer(0.1, self.callback_timer)

    def callback_timer(self):
        try:
            color = self.scene_data.color
            depth = self.scene_data.depth
            if color is None or depth is None:
                return

            color_aligned, depth_aligned = self.align_color_and_depth(color, depth)
            if color_aligned is None:
                return

            self.scene_data.color_align_depth = color_aligned
            self.scene_data.depth_aligned = depth_aligned

            masks, result_img = self.sam_wrapper.get_masks(color_aligned, dilate_masks=True, show_result_img=True)
            if result_img is not None:
                cv2.imshow('Result Image', result_img)
                cv2.waitKey(1)
            else:
                self.get_logger().error('Failed to get result image')
                return

            # 각 마스크에 depth 값 추가 + 마스크 밖 depth 추출
            masks, depth_without_mask = self._assign_depth_to_masks(masks, depth_aligned)

            # 마스크별 포인트클라우드 → 형태 추정
            shape_results = self._estimate_mask_shapes(masks, depth_aligned)
            for i, sr in enumerate(shape_results):
                self.get_logger().info(
                    f'  mask[{i}] {sr.shape.value} '
                    f'conf={sr.confidence:.2f} dim={sr.dimensions}'
                )

            self.scene_data.masks = masks
            self.scene_data.depth_without_mask = depth_without_mask
            self.scene_data.shape_results = shape_results
            self.scene_data.result_img = result_img

            # ShapeResult → RViz MarkerArray
            self._publish_shape_markers(shape_results)

            # ShapeResult → MoveIt2 CollisionObject → PlanningScene publish
            self._publish_collision_objects(shape_results)

            # depth_without_mask → Open3D voxel → PointCloud2 퍼블리시
            pts = self._depth_to_voxel_pcd(depth_without_mask)
            # if pts is not None and len(pts) > 0:
            #     self.scene_data.pcd_without_mask = pts
            #     header = self.scene_data.depth_header
            #     self.pub_background_cloud.publish(numpy_to_ros(pts, header))
            #     self.get_logger().info(f'background cloud: {len(pts)} pts')

        except Exception as e:
            self.get_logger().error(f'Error in callback_timer: {e}')

    def _deproject_mask_to_points(
        self, seg: np.ndarray, depth_aligned: np.ndarray
    ) -> 'np.ndarray | None':
        """
        마스크 segmentation(bool H×W) + depth(uint16 mm) → (K, 3) float32

        depth_aligned 해상도와 _depth_intrinsics 해상도가 다를 경우
        intrinsics를 비율로 스케일해서 사용.
        반환 좌표계: 카메라 좌표 (x, y, z=depth[m])
        """
        if self._depth_intrinsics is None:
            return None

        iw = self._depth_intrinsics.width
        ih = self._depth_intrinsics.height
        dh, dw = depth_aligned.shape

        K  = self._depth_intrinsics.intrinsic_matrix
        fx = K[0, 0] * (dw / iw)
        fy = K[1, 1] * (dh / ih)
        cx = K[0, 2] * (dw / iw)
        cy = K[1, 2] * (dh / ih)

        vs, us = np.where(seg)
        zs = depth_aligned[vs, us].astype(np.float32) / self.cfg.depth_scale  # mm→m

        valid = (zs > 0) & (zs < self.cfg.depth_trunc)
        if valid.sum() < 50:
            return None

        us = us[valid].astype(np.float32)
        vs = vs[valid].astype(np.float32)
        zs = zs[valid]

        X = (us - cx) / fx * zs
        Y = (vs - cy) / fy * zs
        return np.stack([X, Y, zs], axis=-1)   # (K, 3) float32

    def _estimate_mask_shapes(
        self, masks: list, depth_aligned: np.ndarray
    ) -> 'list[ShapeResult]':
        """
        각 마스크 → 3D 역투영 + 2D 마스크 → estimate_shape 호출

        결과는 mask['shape'] 에도 저장됨.
        반환: List[ShapeResult]  (masks 순서와 동일)
        """
        results = []
        for mask in masks:
            seg    = mask['segmentation']
            points = self._deproject_mask_to_points(seg, depth_aligned)
            if points is None:
                sr = ShapeResult(
                    shape=ShapeType.UNKNOWN, confidence=0.0,
                    center=np.zeros(3), dimensions={}, scores={}
                )
            else:
                sr = estimate_shape(points, seg)
            mask['shape'] = sr
            results.append(sr)
        return results

    @staticmethod
    def _obb_to_quaternion(obb_R: np.ndarray) -> Quaternion:
        """OBB rotation matrix (3×3) → geometry_msgs/Quaternion"""
        xyzw = SciRot.from_matrix(obb_R).as_quat()   # scipy: [x, y, z, w]
        return Quaternion(x=xyzw[0], y=xyzw[1], z=xyzw[2], w=xyzw[3])

    @staticmethod
    def _align_z_to_axis(target: np.ndarray) -> Quaternion:
        """
        RViz CYLINDER/ARROW의 기본 축(로컬 Z)을 target 벡터 방향으로 회전시키는 쿼터니언.
        target: (3,) 단위 벡터 (카메라 프레임 기준 긴 축 방향)
        """
        z = np.array([0.0, 0.0, 1.0])
        v = target / (np.linalg.norm(target) + 1e-9)

        dot = np.clip(np.dot(z, v), -1.0, 1.0)
        axis = np.cross(z, v)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-6:
            # 평행 (같은 방향 or 반대)
            if dot > 0:
                return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            else:
                # 180° 회전 — X 축 기준
                return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        axis = axis / axis_norm
        angle = np.arccos(dot)
        xyzw = SciRot.from_rotvec(angle * axis).as_quat()
        return Quaternion(x=xyzw[0], y=xyzw[1], z=xyzw[2], w=xyzw[3])

    # 형태별 색상 (RGBA 0~1)
    _SHAPE_COLORS = {
        ShapeType.SPHERE:   (0.2, 0.8, 0.3, 0.7),   # 초록
        ShapeType.CYLINDER: (0.2, 0.5, 1.0, 0.7),   # 파랑
        ShapeType.CUBE:     (1.0, 0.6, 0.1, 0.7),   # 주황
        ShapeType.UNKNOWN:  (0.5, 0.5, 0.5, 0.3),   # 회색
    }

    def _publish_shape_markers(self, shape_results: list):
        """
        ShapeResult → visualization_msgs/MarkerArray

        마커 구성 (마스크당 2개):
          [짝수 id] 형상 마커  : SPHERE / CYLINDER / CUBE
          [홀수 id] 텍스트 라벨: "sphere 0.82 r=0.12m"

        토픽: /scene/shape_markers
        RViz 설정: Add → MarkerArray → Topic: /scene/shape_markers
        """
        if not shape_results:
            return

        now      = self.get_clock().now().to_msg()
        src_frame = self._depth_frame_id or 'camera_depth_optical_frame'
        ma = MarkerArray()

        # 이전 프레임 마커 전체 삭제
        del_mk        = Marker()
        del_mk.action = Marker.DELETEALL
        del_mk.ns     = 'shapes'
        ma.markers.append(del_mk)

        for idx, sr in enumerate(shape_results):
            if sr.shape == ShapeType.UNKNOWN:
                continue
            if sr.confidence < self.cfg.min_shape_confidence:
                continue

            r, g, b, a = self._SHAPE_COLORS[sr.shape]
            cx, cy, cz = float(sr.center[0]), float(sr.center[1]), float(sr.center[2])

            # ── 형상 마커 ────────────────────────────────────────────
            mk           = Marker()
            mk.header.frame_id = src_frame
            mk.header.stamp    = now
            mk.ns        = 'shapes'
            mk.id        = idx * 2
            mk.action    = Marker.ADD
            mk.pose.position.x = cx
            mk.pose.position.y = cy
            mk.pose.position.z = cz
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, a
            mk.lifetime.sec = 1   # 1초 후 자동 소멸 (프레임 드롭 시 잔상 방지)

            if sr.shape == ShapeType.SPHERE:
                mk.type    = Marker.SPHERE
                rad        = sr.dimensions.get('radius', 0.1)
                mk.scale.x = mk.scale.y = mk.scale.z = rad * 2.0
                mk.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            elif sr.shape == ShapeType.CYLINDER:
                mk.type    = Marker.CYLINDER
                mk.scale.x = mk.scale.y = sr.dimensions.get('radius', 0.05) * 2.0
                mk.scale.z = sr.dimensions.get('height', 0.2)
                # CYLINDER 기본축(로컬 Z)을 OBB 가장 긴 축 방향으로 회전
                if sr.main_axis is not None:
                    mk.pose.orientation = self._align_z_to_axis(sr.main_axis)
                else:
                    mk.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            else:  # CUBE
                mk.type    = Marker.CUBE
                mk.scale.x = sr.dimensions.get('width',  0.1)
                mk.scale.y = sr.dimensions.get('depth',  0.1)
                mk.scale.z = sr.dimensions.get('height', 0.1)
                # CUBE를 OBB rotation으로 정렬
                if sr.obb_R is not None:
                    mk.pose.orientation = self._obb_to_quaternion(sr.obb_R)
                else:
                    mk.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            ma.markers.append(mk)

            # ── 텍스트 라벨 ──────────────────────────────────────────
            dim_str = '  '.join(f'{k}={v:.2f}' for k, v in sr.dimensions.items())
            label            = Marker()
            label.header     = mk.header
            label.ns         = 'shapes'
            label.id         = idx * 2 + 1
            label.type       = Marker.TEXT_VIEW_FACING
            label.action     = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = cz + mk.scale.z / 2.0 + 0.05
            label.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            label.scale.z    = 0.06
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.lifetime.sec = 1
            label.text       = f'{sr.shape.value} {sr.confidence:.2f}\n{dim_str}'
            ma.markers.append(label)

        self.pub_shape_markers.publish(ma)

    def _publish_collision_objects(self, shape_results: list):
        """
        ShapeResult 리스트 → moveit_msgs/CollisionObject 생성 → PlanningScene publish

        [흐름]
          1. 이전 프레임 충돌 객체 전체 삭제 (REMOVE)
          2. 각 ShapeResult를 SolidPrimitive(BOX/SPHERE/CYLINDER)로 변환
          3. center를 TF로 planning_frame으로 변환
          4. PlanningScene.world.collision_objects 에 모두 담아 한 번에 publish

        TF를 찾을 수 없으면 카메라 프레임 그대로 사용.
        """
        if not shape_results:
            return

        now = self.get_clock().now().to_msg()
        src_frame = self._depth_frame_id or 'camera_depth_optical_frame'
        dst_frame = self.cfg.planning_frame

        # TF 변환 시도 (실패 시 None → 카메라 프레임 사용)
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                dst_frame, src_frame, rclpy.time.Time()
            )
        except Exception:
            tf_stamped = None

        collision_objects = []

        for idx, sr in enumerate(shape_results):
            if sr.shape == ShapeType.UNKNOWN:
                continue
            if sr.confidence < self.cfg.min_shape_confidence:
                continue

            # ── SolidPrimitive ─────────────────────────────────────
            prim = SolidPrimitive()
            if sr.shape == ShapeType.SPHERE:
                prim.type = SolidPrimitive.SPHERE
                prim.dimensions = [sr.dimensions.get('radius', 0.1)]

            elif sr.shape == ShapeType.CYLINDER:
                prim.type = SolidPrimitive.CYLINDER
                # CYLINDER: [height, radius]  (SolidPrimitive 순서)
                prim.dimensions = [
                    sr.dimensions.get('height', 0.2),
                    sr.dimensions.get('radius', 0.05),
                ]
            else:  # CUBE / BOX
                prim.type = SolidPrimitive.BOX
                # BOX: [x(width), y(depth), z(height)]
                prim.dimensions = [
                    sr.dimensions.get('width',  0.1),
                    sr.dimensions.get('depth',  0.1),
                    sr.dimensions.get('height', 0.1),
                ]

            # ── Pose (center 변환) ──────────────────────────────────
            center_stamped = PoseStamped()
            center_stamped.header.frame_id = src_frame
            center_stamped.header.stamp    = now
            center_stamped.pose.position.x = float(sr.center[0])
            center_stamped.pose.position.y = float(sr.center[1])
            center_stamped.pose.position.z = float(sr.center[2])
            center_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            if tf_stamped is not None:
                try:
                    transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                        center_stamped, tf_stamped
                    )
                    pose = transformed.pose
                    frame_id = dst_frame
                except Exception:
                    pose     = center_stamped.pose
                    frame_id = src_frame
            else:
                pose     = center_stamped.pose
                frame_id = src_frame

            # ── CollisionObject 조립 ────────────────────────────────
            co = CollisionObject()
            co.header.frame_id = frame_id
            co.header.stamp    = now
            co.id              = f'obj_{idx}'
            co.operation       = CollisionObject.ADD
            co.primitives.append(prim)
            co.primitive_poses.append(pose)
            collision_objects.append(co)

        if not collision_objects:
            return

        # ── PlanningScene으로 한 번에 전송 ──────────────────────────
        # is_diff=True: 기존 scene에 diff(추가/수정)만 적용
        ps = PlanningScene()
        ps.is_diff = True

        # 이전 프레임 객체 전체 삭제
        clear_all = CollisionObject()
        clear_all.header.frame_id = dst_frame
        clear_all.header.stamp    = now
        clear_all.id              = '__clear_scene__'
        clear_all.operation       = CollisionObject.REMOVE
        ps.world.collision_objects.append(clear_all)

        ps.world.collision_objects.extend(collision_objects)
        self.pub_planning_scene.publish(ps)
        self.get_logger().info(
            f'CollisionObjects published: {len(collision_objects)} '
            f'→ {dst_frame}'
        )

    def _assign_depth_to_masks(self, masks, depth_aligned):
        """
        각 마스크에 해당 영역의 depth 배열을 추가하고,
        어떤 마스크에도 속하지 않는 depth 픽셀을 depth_without_mask로 반환.

        mask['depth']       : (K,) uint16  — 해당 마스크 영역의 depth 값 (K = 마스크 픽셀 수)
        mask['depth_mean']  : float        — 마스크 영역 유효 depth(>0) 평균값 (mm)
        depth_without_mask  : (H, W) uint16 — 마스크 밖 depth, 마스크 안은 0
        """
        if not masks or depth_aligned is None:
            return masks, depth_aligned

        combined_mask = np.zeros(depth_aligned.shape, dtype=bool)

        for mask in masks:
            seg = mask['segmentation']              # (H, W) bool
            depth_vals = depth_aligned[seg]         # (K,) uint16
            mask['depth'] = depth_vals
            valid = depth_vals[depth_vals > 0]
            mask['depth_mean'] = float(valid.mean()) if len(valid) > 0 else 0.0
            combined_mask |= seg

        depth_without_mask = depth_aligned.copy()
        depth_without_mask[combined_mask] = 0       # 마스크 영역은 0으로

        return masks, depth_without_mask
    
    def callback_depth_info(self, msg: CameraInfo):
        """depth 카메라 내부 파라미터 수신 (최초 1회만 설정)"""
        if self._depth_intrinsics is not None:
            return
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        w, h   = msg.width, msg.height
        self._depth_intrinsics = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
        self._depth_frame_id   = msg.header.frame_id
        self.get_logger().info(
            f'Depth intrinsics 설정: {w}×{h}  fx={fx:.1f} fy={fy:.1f} '
            f'frame_id={self._depth_frame_id}'
        )

    def _depth_to_voxel_pcd(self, depth_without_mask: np.ndarray) -> 'np.ndarray | None':
        """
        depth_without_mask (원본 depth 해상도, uint16, mm 단위) →
          1. Open3D DepthImage → PointCloud (deproject)
          2. voxel_down_sample
          3. (N, 3) float32 반환

        카메라 내부 파라미터(_depth_intrinsics)가 없으면 None 반환.
        depth_without_mask의 해상도가 intrinsics와 다를 경우 자동 resize.
        """
        if self._depth_intrinsics is None:
            return None

        iw = self._depth_intrinsics.width
        ih = self._depth_intrinsics.height
        dh, dw = depth_without_mask.shape

        # 해상도 불일치 시 intrinsics 해상도로 맞춤
        if dw != iw or dh != ih:
            depth_rs = cv2.resize(
                depth_without_mask, (iw, ih), interpolation=cv2.INTER_NEAREST
            )
        else:
            depth_rs = depth_without_mask

        depth_o3d = o3d.geometry.Image(depth_rs.astype(np.uint16))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d,
            self._depth_intrinsics,
            depth_scale=self.cfg.depth_scale,   # mm → m
            depth_trunc=self.cfg.depth_trunc,   # 최대 거리 (m)
        )

        pcd_ds = pcd.voxel_down_sample(self.cfg.voxel_size)
        pts = np.asarray(pcd_ds.points, dtype=np.float32)
        return pts if len(pts) > 0 else None

    def callback_rgb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8).copy()  # frombuffer → read-only
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.scene_data.color = image
        except Exception as e:
            self.get_logger().error(f'Error in callback_rgb: {e}')
            return

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
            self.scene_data.depth = depth
            self.scene_data.depth_header = msg.header
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

    def set_config(self):
        self.cfg = Config()

        self.declare_parameter('input_rgb', '/femto/color/image_raw/compressed')
        self.declare_parameter('input_depth', '/femto/depth/image_raw/compressedDepth')
        self.declare_parameter('input_depth_info', '/femto/depth/camera_info')
        self.declare_parameter('input_pcd', '/femto/depth_registered/points')
        self.declare_parameter('sam2_model', 'sam2.1_hiera_large.pt')
        self.declare_parameter('sam2_cfg', 'sam2.1_hiera_l.yaml')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('depth_trunc', 5.0)
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('min_shape_confidence', 0.4)

        self.cfg.input_rgb             = self.get_parameter('input_rgb').value
        self.cfg.input_depth           = self.get_parameter('input_depth').value
        self.cfg.input_depth_info      = self.get_parameter('input_depth_info').value
        self.cfg.input_pcd             = self.get_parameter('input_pcd').value
        self.cfg.sam2_model            = self.get_parameter('sam2_model').value
        self.cfg.sam2_cfg              = self.get_parameter('sam2_cfg').value
        self.cfg.voxel_size            = self.get_parameter('voxel_size').value
        self.cfg.depth_scale           = self.get_parameter('depth_scale').value
        self.cfg.depth_trunc           = self.get_parameter('depth_trunc').value
        self.cfg.planning_frame        = self.get_parameter('planning_frame').value
        self.cfg.min_shape_confidence  = self.get_parameter('min_shape_confidence').value

    def align_color_and_depth(self, color, depth):
        """
        depth를 color 해상도로 resize하고, depth=0 픽셀을 color에서 마스킹.
        반환: (color_masked, depth_resized)
        """
        try:
            if color is None or depth is None:
                return None, None

            depth_resized = cv2.resize(
                depth, (color.shape[1], color.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )
            mask_invalid = depth_resized == 0
            color_masked = color.copy()
            color_masked[mask_invalid] = 0

            return color_masked, depth_resized

        except Exception as e:
            self.get_logger().error(f'Error in align_color_and_depth: {e}')
            return None, None

def main(args=None):
    import os
    import sam2 as _sam2_pkg
    from pathlib import Path
    # Hydra가 outputs/ 디렉토리를 쓰기 위해 SAM2 소스 루트로 이동
    os.chdir(Path(_sam2_pkg.__file__).resolve().parent.parent)

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
