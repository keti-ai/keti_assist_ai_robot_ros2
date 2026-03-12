import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros_rgb

"""
=============================================================================
Process : PointCloud 파이프라인 처리 클래스  (Open3D 기반)
=============================================================================
ObstacleRepresentation.__init__ 에서 Process(self) 로 생성하여 사용.
노드 속성(설정값, publisher 등)은 self._node 를 통해 접근.

[파이프라인]
  1. preprocess       : np.frombuffer 직접 파싱 + ROI 필터
  2. downsample_voxel : o3d.voxel_down_sample  (C++, 빠름)
  3. remove_ground    : Z-percentile 필터      (numpy, O(N))
  4. cluster_o3d      : o3d.cluster_dbscan     (C++, 빠름)
  5. compute_aabb     : numpy AABB 계산
  6. collision_and_visualize : 충돌 판정 + RViz2 퍼블리시

[이전 대비 변경]
  - VoxelGrid(Python dict BFS)  →  Open3D C++ 루틴
  - cluster_bfs(Python deque)   →  cluster_dbscan (C++)
  - build_voxel_grid            →  downsample_voxel
  - compute_aabb(voxel keys)    →  compute_aabb(pts + labels)
=============================================================================
"""

# 클러스터 시각화 색상 팔레트 (RGB 0~1)
CLUSTER_COLORS = [
    (1.0, 0.2, 0.2),   # red
    (0.2, 1.0, 0.2),   # green
    (0.2, 0.2, 1.0),   # blue
    (1.0, 1.0, 0.2),   # yellow
    (1.0, 0.2, 1.0),   # magenta
    (0.2, 1.0, 1.0),   # cyan
    (1.0, 0.6, 0.2),   # orange
    (0.6, 0.2, 1.0),   # purple
]


class Process:
    """
    _node 를 통해 접근하는 속성 (ObstacleRepresentation._bind_cfg 에서 초기화):
      voxel_size, roi_x/y/z_min/max, ground_height_thr, ground_z_percentile,
      min/max_cluster_voxels, robot_pos, robot_safety_radius,
      pub_bbox, pub_cluster_cloud, pub_collision
    """

    def __init__(self, node):
        self._node = node

    # ================================================================
    # 1단계: 전처리
    # ================================================================
    def preprocess(self, msg: PointCloud2) -> 'np.ndarray | None':
        """
        ROI PassThrough 필터 + NaN/Inf 제거
        반환: (N, 3) float32 numpy array, 또는 None

        msg.data 를 structured dtype 으로 직접 파싱 → Python 루프 없음
        """
        n = self._node
        try:
            off = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z')}
            dt = np.dtype({
                'names':   ['x', 'y', 'z'],
                'formats': [np.float32, np.float32, np.float32],
                'offsets': [off['x'], off['y'], off['z']],
                'itemsize': msg.point_step,
            })
            arr = np.frombuffer(bytes(msg.data), dtype=dt)
            if len(arr) == 0:
                return None
            pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
        except Exception as e:
            n.get_logger().warn(f'PointCloud read error: {e}')
            return None

        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return None

        mask = (
            (pts[:, 0] >= n.roi_x_min) & (pts[:, 0] <= n.roi_x_max) &
            (pts[:, 1] >= n.roi_y_min) & (pts[:, 1] <= n.roi_y_max) &
            (pts[:, 2] >= n.roi_z_min) & (pts[:, 2] <= n.roi_z_max)
        )
        filtered = pts[mask]
        return filtered if len(filtered) > 0 else None

    # ================================================================
    # 2단계: Voxel 다운샘플링  (구: build_voxel_grid)
    # ================================================================
    def downsample_voxel(self, points: np.ndarray) -> np.ndarray:
        """
        o3d.voxel_down_sample 기반 다운샘플링
        반환: (M, 3) float32  (M << N)
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        ds = pcd.voxel_down_sample(self._node.voxel_size)
        return np.asarray(ds.points, dtype=np.float32)

    # ================================================================
    # 2.5단계: 포인트 → 메시 (TRIANGLE_LIST Marker)
    # ================================================================
    def create_mesh_marker(self, points: np.ndarray, header) -> 'Marker | None':
        """
        (M, 3) float32 포인트 → RViz2 TRIANGLE_LIST Marker

        [방식] Open3D Ball Pivoting Algorithm (BPA)
          - 법선 추정 → BPA 로 삼각형 표면 재구성
          - voxel_size 기반 ball 반지름 자동 설정

        [대안] 포인트가 너무 적거나 BPA 실패 시 Convex Hull 로 fallback
        """
        n = self._node
        if len(points) < 4:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

        # 법선 추정 (BPA 필수)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=n.voxel_size * 3, max_nn=30
            )
        )
        pcd.orient_normals_consistent_tangent_plane(k=15)

        # Ball Pivoting: voxel_size 배수로 여러 반지름 시도
        vs = n.voxel_size
        radii = o3d.utility.DoubleVector([vs, vs * 2, vs * 4])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, radii
        )
        # BPA 결과가 없으면 Convex Hull 로 fallback
        if len(mesh.triangles) == 0:
            mesh, _ = pcd.compute_convex_hull()

        mesh.orient_triangles()          # winding order 통일 (검정 음영 방지)
        mesh.compute_vertex_normals()    # 법선 재계산

        # mesh, _ = pcd.compute_convex_hull()

        verts = np.asarray(mesh.vertices)
        tris  = np.asarray(mesh.triangles)
        if len(tris) == 0:
            return None

        # 양면 렌더링: 뒤집힌 winding order 삼각형을 함께 추가
        # RViz2 TRIANGLE_LIST는 단면 렌더링이므로 검정 음영 제거에 필수
        tris_double = np.vstack([tris, tris[:, ::-1]])

        mk = Marker()
        mk.header    = header
        mk.ns        = 'point_mesh'
        mk.id        = 0
        mk.type      = Marker.TRIANGLE_LIST
        mk.action    = Marker.ADD
        mk.scale.x   = mk.scale.y = mk.scale.z = 1.0
        mk.color.r   = 1.0
        mk.color.g   = 1.0
        mk.color.b   = 1.0
        mk.color.a   = 0.5

        for tri in tris_double:
            for vi in tri:
                p = Point()
                p.x, p.y, p.z = float(verts[vi, 0]), float(verts[vi, 1]), float(verts[vi, 2])
                mk.points.append(p)

        return mk

    def create_shape_markers(
        self,
        centers: np.ndarray,
        shape: str,
        frame_id: str,
        stamp,
        color: tuple = (1.0, 1.0, 1.0, 1.0),  # RGBA
    ) -> MarkerArray:
        """
        구 중심 리스트 → MarkerArray

        SPHERE_LIST 방식: 마커 하나에 구 여러 개 → 퍼블리시 오버헤드 최소화
        """
        marker_array = MarkerArray()

        # 기존 마커 삭제 (DELETE_ALL)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if len(centers) == 0:
            return marker_array



        # SPHERE_LIST: 구 여러 개를 마커 하나로 표현 (효율적)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'obstacle_spheres'
        marker.id = 0
        if shape == 'sphere':
            marker.type = Marker.SPHERE_LIST
        elif shape == 'cube':
            marker.type = Marker.CUBE_LIST
        elif shape == 'cylinder':
            marker.type = Marker.CYLINDER_LIST
        elif shape == 'cone':
            marker.type = Marker.CONE_LIST
        elif shape == 'pyramid':
            marker.type = Marker.PYRAMID_LIST
        marker.action = Marker.ADD

        # 구 크기
        marker.scale.x = self._node.voxel_size * 1  # 지름 = 반지름 * 2
        marker.scale.y = self._node.voxel_size * 1
        marker.scale.z = self._node.voxel_size * 1

        # 색상 (RGBA)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # 구 중심 좌표 추가
        for center in centers:
            p = Point()
            p.x = float(center[0])
            p.y = float(center[1])
            p.z = float(center[2])
            marker.points.append(p)

        marker_array.markers.append(marker)
        return marker_array

    # ================================================================
    # CollisionObject 생성 (MoveIt2 Planning Scene용)
    # ================================================================
    def create_collision_object(
        self,
        centers: np.ndarray,
        frame_id: str,
        stamp,
        shape: str = 'sphere',
        object_id: str = 'obstacle_cloud',
    ) -> CollisionObject:
        """
        다운샘플된 포인트 배열 → MoveIt2 CollisionObject

        각 voxel 중심을 지정한 shape 으로 등록하여
        /collision_object 토픽으로 퍼블리시하면 PlanningSceneMonitor가
        플래닝 씬에 즉시 반영한다.

        Args:
            centers   : (N, 3) float32 - 중심 좌표 배열
            frame_id  : 좌표계 프레임 ID
            stamp     : 타임스탬프
            shape     : 'sphere' | 'cube'
                          sphere → 반지름 = voxel_size / 2
                                   dimensions = [radius]
                          cube   → 한 변 = voxel_size
                                   dimensions = [x, y, z]
            object_id : CollisionObject 식별자 (같은 id로 재전송 시 덮어씀)

        Returns:
            moveit_msgs/CollisionObject
        """
        n = self._node
        vs = float(n.voxel_size)

        if shape == 'sphere':
            prim_type = SolidPrimitive.SPHERE
            dimensions = [vs / 2.0]          # [radius]
        elif shape == 'cube':
            prim_type = SolidPrimitive.BOX
            dimensions = [vs, vs, vs]        # [x, y, z]
        else:
            raise ValueError(f"shape must be 'sphere' or 'cube', got '{shape}'")

        co = CollisionObject()
        co.header.frame_id = frame_id
        co.header.stamp = stamp
        co.id = object_id
        co.operation = CollisionObject.ADD

        for center in centers:
            primitive = SolidPrimitive()
            primitive.type = prim_type
            primitive.dimensions = dimensions
            co.primitives.append(primitive)

            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)

        return co
