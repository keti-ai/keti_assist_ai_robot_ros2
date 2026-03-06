import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA, Header
from geometry_msgs.msg import Point, Vector3

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import struct
import time

from kaair_obstacle.utils.voxel_grid import VoxelGrid
from kaair_obstacle.utils.load_config import load_config, PipelineConfig
from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros, numpy_to_ros_rgb

"""
=============================================================================
Voxel-based 3D Collision Avoidance Pipeline
=============================================================================
Orbbec Camera PointCloud2 → Voxel Grid → Clustering → AABB → Collision Check

[파이프라인]
  1. 전처리      : ROI 필터 + Voxel Downsampling + Outlier 제거
  2. Voxel Grid  : 3D Occupancy Grid 생성 (dict 기반 sparse representation)
  3. 바닥 제거   : Z 히스토그램 분석으로 ground plane voxel 제거
  4. 클러스터링  : 26-연결 BFS (인접 voxel 탐색, 거리계산 불필요)
  5. AABB        : 각 클러스터의 축정렬 Bounding Box 생성
  6. Collision   : 로봇 안전 구역과 AABB 교차 판정
  7. 시각화      : RViz2 MarkerArray 퍼블리시

[퍼블리시 토픽]
  /pointcloud/preprocessed     - 전처리된 PointCloud2
  /pointcloud/no_ground        - 바닥 제거된 PointCloud2
  /voxel/occupied              - Occupied voxel 시각화 (MarkerArray)
  /objects/bounding_boxes      - AABB 박스 (MarkerArray)
  /objects/cluster_cloud       - 클러스터별 색상 PointCloud2
  /collision/status            - 충돌 위험 상태 (String)
=============================================================================
"""


# ============================================================================
# 색상 팔레트 (클러스터 시각화용)
# ============================================================================
CLUSTER_COLORS = [
    (1.0, 0.2, 0.2),  # red
    (0.2, 1.0, 0.2),  # green
    (0.2, 0.2, 1.0),  # blue
    (1.0, 1.0, 0.2),  # yellow
    (1.0, 0.2, 1.0),  # magenta
    (0.2, 1.0, 1.0),  # cyan
    (1.0, 0.6, 0.2),  # orange
    (0.6, 0.2, 1.0),  # purple
]


class ObstacleRepresentation(Node):
    def __init__(self):
        super().__init__('obstacle_representation')

        self.cfg: PipelineConfig = load_config(self)
        self.get_logger().info(f'PipelineConfig: {self.cfg}')
        # ----------------------------------------------------------------
        # VoxelGrid 객체
        # ----------------------------------------------------------------
        self.voxel_grid = VoxelGrid(self.voxel_size)

        # ----------------------------------------------------------------
        # QoS 설정 (Orbbec 카메라 호환)
        # ----------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------------------
        # Subscriber
        # ----------------------------------------------------------------
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            sensor_qos
        )

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------
        self.pub_preprocessed = self.create_publisher(
            PointCloud2, '/pointcloud/preprocessed', 10)
        self.pub_no_ground = self.create_publisher(
            PointCloud2, '/pointcloud/no_ground', 10)
        self.pub_voxels = self.create_publisher(
            MarkerArray, '/voxel/occupied', 10)
        self.pub_bbox = self.create_publisher(
            MarkerArray, '/objects/bounding_boxes', 10)
        self.pub_cluster_cloud = self.create_publisher(
            PointCloud2, '/objects/cluster_cloud', 10)
        self.pub_collision = self.create_publisher(
            String, '/collision/status', 10)

        self.get_logger().info(
            f'VoxelPipelineNode started | voxel_size={self.voxel_size}m | '
            f'topic={self.input_topic}'
        )
        self.frame_count = 0
        
        self.get_logger().info('ObstacleRepresentation node initialized')

    def callback(self, msg):
        st = time.time()
        self.frame_count += 1

        points = self.preprocess(msg)
        if points is not None or len(points) < 10:
            return 

        self.pub_preprocessed.publish(numpy_to_ros(points, msg.header))


    # ================================================================
    # 1단계: 전처리
    # ================================================================
    def preprocess(self, msg: PointCloud2) -> np.ndarray:
        """
        ROI 박스 필터링
        NaN/Inf 제거
        반환: (N, 3) float32 numpy array
        """
        try:
            gen = pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.array(list(gen), dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'PointCloud read error: {e}')
            return None

        if len(pts) == 0:
            return None

        # Inf 제거
        pts = pts[np.isfinite(pts).all(axis=1)]

        # ROI 박스 필터 (벡터화 마스킹)
        mask = (
            (pts[:, 0] >= self.roi_x_min) & (pts[:, 0] <= self.roi_x_max) &
            (pts[:, 1] >= self.roi_y_min) & (pts[:, 1] <= self.roi_y_max) &
            (pts[:, 2] >= self.roi_z_min) & (pts[:, 2] <= self.roi_z_max)
        )
        return pts[mask]

    # ================================================================
    # 2단계: Voxel Grid 생성
    # ================================================================
    def build_voxel_grid(self, points: np.ndarray):
        """
        포인트 → Sparse Voxel Grid
        각 voxel에 포인트들의 centroid 저장
        """
        self.voxel_grid.insert_points(points)

    # ================================================================
    # 3단계: 바닥 제거 (Z 히스토그램 기반)
    # ================================================================
    def remove_ground(self) -> set:
        """
        Voxel Z 인덱스 히스토그램으로 바닥 레이어 탐지
        
        원리:
          - 바닥은 넓고 평평하므로 특정 Z 인덱스에 voxel이 집중됨
          - 최저 Z 레이어부터 ground_height_threshold 범위를 바닥으로 판정
        """
        if not self.voxel_grid.voxels:
            return set()

        keys = self.voxel_grid.get_occupied_keys()
        z_indices = np.array([k[2] for k in keys])

        # 최저 Z 인덱스 기준으로 바닥 두께 계산
        z_min_idx = np.percentile(z_indices, self.ground_z_percentile)
        ground_thickness = int(np.ceil(
            self.ground_height_thr / self.voxel_size))

        # 바닥 voxel 키 수집
        ground_keys = set(
            k for k in keys
            if k[2] <= z_min_idx + ground_thickness
        )

        self.voxel_grid.remove_keys(ground_keys)
        return ground_keys


    # ================================================================
    # 4단계: BFS 클러스터링
    # ================================================================
    def cluster_bfs(self) -> list:
        """
        26-연결 BFS (Breadth-First Search) 클러스터링
        
        원리:
          - 각 voxel을 node로, 인접 voxel을 edge로 보는 그래프
          - 방문하지 않은 voxel에서 BFS 시작 → 하나의 클러스터
          - 26-연결: 각 voxel의 3x3x3-1 = 26개 이웃 탐색
          
        장점:
          - Euclidean 거리 계산 불필요 (인덱스 산술만 사용)
          - 시간복잡도: O(N) where N = occupied voxels
        """
        occupied = set(self.voxel_grid.get_occupied_keys())
        visited = set()
        clusters = []  # list of list of keys

        # 26-연결 오프셋 생성 (3x3x3 큐브에서 자기 자신 제외)
        if self.connectivity == 26:
            offsets = [
                (dx, dy, dz)
                for dx in [-1, 0, 1]
                for dy in [-1, 0, 1]
                for dz in [-1, 0, 1]
                if not (dx == 0 and dy == 0 and dz == 0)
            ]
        else:  # 6-연결 (face-adjacent only)
            offsets = [
                (-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)
            ]

        for start_key in occupied:
            if start_key in visited:
                continue

            # BFS 시작
            cluster = []
            queue = deque([start_key])
            visited.add(start_key)

            while queue:
                key = queue.popleft()
                cluster.append(key)

                # 26개 이웃 탐색
                kx, ky, kz = key
                for dx, dy, dz in offsets:
                    neighbor = (kx + dx, ky + dy, kz + dz)
                    if neighbor in occupied and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            # 크기 필터 (너무 작거나 너무 큰 클러스터 제외)
            if self.min_cluster_voxels <= len(cluster) <= self.max_cluster_voxels:
                clusters.append(cluster)

        # 크기 내림차순 정렬
        clusters.sort(key=lambda c: len(c), reverse=True)
        return clusters

    # ================================================================
    # 5단계: AABB 계산
    # ================================================================
    def compute_aabb(self, clusters: list) -> list:
        """
        각 클러스터에 대해 Axis-Aligned Bounding Box 계산
        
        반환: list of dict {
            'min': np.array([x,y,z]),
            'max': np.array([x,y,z]),
            'center': np.array([x,y,z]),
            'size': np.array([w,h,d]),
            'num_voxels': int,
            'points': np.array  - 클러스터 포인트들
        }
        
        Voxel 기반 AABB:
          - 각 voxel의 min/max 코너를 직접 사용
          - voxel_size 단위로 정렬되어 박스가 깔끔함
        """
        bboxes = []
        vs = self.voxel_size

        for cluster_keys in clusters:
            # voxel 인덱스 배열로 변환
            key_arr = np.array(cluster_keys, dtype=np.int32)

            # voxel 코너 좌표 (min/max)
            min_idx = key_arr.min(axis=0)
            max_idx = key_arr.max(axis=0)

            # 실수 좌표로 변환 (voxel 경계 기준)
            bb_min = min_idx.astype(float) * vs
            bb_max = (max_idx + 1).astype(float) * vs
            bb_center = (bb_min + bb_max) / 2.0
            bb_size = bb_max - bb_min

            # 클러스터 포인트 centroid 수집
            pts = np.array([
                self.voxel_grid.voxels[k]
                for k in cluster_keys
                if k in self.voxel_grid.voxels
            ])

            bboxes.append({
                'min': bb_min,
                'max': bb_max,
                'center': bb_center,
                'size': bb_size,
                'num_voxels': len(cluster_keys),
                'points': pts
            })

        return bboxes

    # ================================================================
    # 6단계: Collision Check + 시각화
    # ================================================================
    def collision_and_visualize(
            self, bboxes: list, clusters: list, header):
        """
        AABB vs 로봇 안전 구역 교차 판정
        + RViz2 MarkerArray 퍼블리시
        """
        bbox_markers = MarkerArray()
        cluster_points_all = []
        cluster_colors_all = []

        collision_objects = []

        for i, bbox in enumerate(bboxes):
            color_idx = i % len(CLUSTER_COLORS)
            r, g, b = CLUSTER_COLORS[color_idx]

            # ── Collision Check ──────────────────────────────────
            # AABB vs 구(sphere) 교차 판정
            # 로봇 위치에서 AABB까지의 최단거리 계산
            closest = np.clip(
                self.robot_pos, bbox['min'], bbox['max'])
            dist = np.linalg.norm(self.robot_pos - closest)

            is_collision = dist < self.robot_safety_radius

            if is_collision:
                collision_objects.append(i)
                r, g, b = 1.0, 0.0, 0.0  # 충돌 위험 → 빨간색

            # ── AABB Marker (LINE_LIST cube) ─────────────────────
            marker = self._make_bbox_marker(
                i, bbox, header, r, g, b, is_collision)
            bbox_markers.markers.append(marker)

            # 라벨 텍스트 마커
            text_marker = self._make_text_marker(
                i + len(bboxes), bbox, header,
                f'#{i} {bbox["num_voxels"]}v {"!" if is_collision else ""}')
            bbox_markers.markers.append(text_marker)

            # ── 클러스터 포인트 수집 ─────────────────────────────
            if len(bbox['points']) > 0:
                cluster_points_all.append(bbox['points'])
                cluster_colors_all.extend(
                    [(r, g, b)] * len(bbox['points']))

        # 이전 프레임 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header = header
        delete_markers = MarkerArray()
        delete_markers.markers.append(delete_marker)

        self.pub_bbox.publish(bbox_markers)

        # ── 클러스터 컬러 포인트클라우드 퍼블리시 ─────────────────
        if cluster_points_all:
            all_pts = np.vstack(cluster_points_all)
            all_colors = np.array(cluster_colors_all)
            self.pub_cluster_cloud.publish(
                self.numpy_to_ros_rgb(all_pts, all_colors, header))

        # ── Collision 상태 퍼블리시 ──────────────────────────────
        status_msg = String()
        if collision_objects:
            status_msg.data = (
                f'DANGER: {len(collision_objects)} object(s) in safety zone '
                f'[ids: {collision_objects}]'
            )
        else:
            status_msg.data = f'SAFE: {len(bboxes)} objects detected'
        self.pub_collision.publish(status_msg)

    # ================================================================
    # 유틸: Marker 생성
    # ================================================================
    def _make_bbox_marker(self, marker_id, bbox, header,
                          r, g, b, is_collision) -> Marker:
        """AABB를 LINE_LIST 형태의 큐브 마커로 생성"""
        marker = Marker()
        marker.header = header
        marker.ns = 'bounding_boxes'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02 if not is_collision else 0.04  # 선 두께

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # AABB 8개 꼭짓점
        mn, mx = bbox['min'], bbox['max']
        corners = [
            [mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]],
            [mx[0], mn[1], mn[2]], [mx[0], mx[1], mn[2]],
            [mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]],
            [mn[0], mx[1], mn[2]], [mn[0], mn[1], mn[2]],
            [mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]],
            [mx[0], mn[1], mx[2]], [mx[0], mx[1], mx[2]],
            [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]],
            [mn[0], mx[1], mx[2]], [mn[0], mn[1], mx[2]],
            [mn[0], mn[1], mn[2]], [mn[0], mn[1], mx[2]],
            [mx[0], mn[1], mn[2]], [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mn[2]], [mx[0], mx[1], mx[2]],
            [mn[0], mx[1], mn[2]], [mn[0], mx[1], mx[2]],
        ]
        for c in corners:
            p = Point()
            p.x, p.y, p.z = float(c[0]), float(c[1]), float(c[2])
            marker.points.append(p)

        return marker

    def _make_text_marker(self, marker_id, bbox, header, text) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(bbox['center'][0])
        marker.pose.position.y = float(bbox['center'][1])
        marker.pose.position.z = float(bbox['max'][2]) + 0.1
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepresentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()