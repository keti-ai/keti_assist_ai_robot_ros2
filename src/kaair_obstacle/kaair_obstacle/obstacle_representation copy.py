import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

import numpy as np

from kaair_obstacle.utils.load_config import load_config, PipelineConfig
from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros
from kaair_obstacle.utils.process import Process

"""
=============================================================================
Voxel-based 3D Collision Avoidance Pipeline  (Open3D 기반)
=============================================================================
Orbbec Camera PointCloud2 → ROI Filter → Voxel Downsample → Ground Removal
                          → DBSCAN Cluster → AABB → Collision Check → RViz2

[파이프라인 단계]
  1. 전처리      : ROI PassThrough + NaN/Inf 제거  (np.frombuffer)
  2. 다운샘플링  : o3d.voxel_down_sample
  3. 바닥 제거   : Z 퍼센타일 필터
  4. 클러스터링  : o3d.cluster_dbscan
  5. AABB        : 각 클러스터의 Axis-Aligned Bounding Box
  6. Collision   : AABB vs 로봇 안전 구역 교차 판정 + RViz2 시각화

[퍼블리시 토픽]
  /pointcloud/preprocessed   - 전처리된 PointCloud2
  /pointcloud/no_ground      - 바닥 제거된 PointCloud2
  /objects/bounding_boxes    - AABB 박스 MarkerArray
  /objects/cluster_cloud     - 클러스터별 컬러 PointCloud2
  /collision/status          - 충돌 위험 상태 String
=============================================================================
"""


class ObstacleRepresentation(Node):

    def __init__(self):
        super().__init__('obstacle_representation_node')

        # ── 설정 로드 ──────────────────────────────────────────────
        self.cfg: PipelineConfig = load_config(self)
        self._bind_cfg()

        # ── 파이프라인 처리 객체 ───────────────────────────────────
        self.process = Process(self)

        # ── QoS (Orbbec BEST_EFFORT 호환) ──────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriber ────────────────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2,
            self.cfg.input_topic,
            self.callback,
            sensor_qos
        )

        # ── Publishers ────────────────────────────────────────────
        self.pub_preprocessed  = self.create_publisher(PointCloud2,  '/pointcloud/preprocessed', 10)
        self.pub_downsampled   = self.create_publisher(PointCloud2,  '/pointcloud/downsampled',  10)
        # self.pub_no_ground     = self.create_publisher(PointCloud2,  '/pointcloud/no_ground',    10)
        self.pub_bbox          = self.create_publisher(MarkerArray,  '/objects/bounding_boxes',  10)
        self.pub_cluster_cloud = self.create_publisher(PointCloud2,  '/objects/cluster_cloud',   10)
        # self.pub_collision     = self.create_publisher(String,       '/collision/status',        10)

        self.frame_count = 0
        self.get_logger().info(
            f'ObstacleRepresentation started | '
            f'topic={self.cfg.input_topic} | voxel={self.cfg.voxel_size}m'
        )

    # ----------------------------------------------------------------
    def _bind_cfg(self):
        """PipelineConfig 값을 self 속성으로 단축 연결"""
        c = self.cfg
        self.voxel_size          = c.voxel_size
        self.roi_x_min           = c.roi_x_min
        self.roi_x_max           = c.roi_x_max
        self.roi_y_min           = c.roi_y_min
        self.roi_y_max           = c.roi_y_max
        self.roi_z_min           = c.roi_z_min
        self.roi_z_max           = c.roi_z_max
        self.ground_height_thr   = c.ground_height_threshold
        self.ground_z_percentile = c.ground_z_percentile
        self.min_cluster_voxels  = c.min_cluster_voxels
        self.max_cluster_voxels  = c.max_cluster_voxels
        self.robot_safety_radius = c.robot_safety_radius
        self.robot_pos           = c.robot_pos              # np.array([x,y,z])

    # ================================================================
    # 메인 콜백
    # ================================================================
    def callback(self, msg: PointCloud2):
        times = [time.time()]
        self.frame_count += 1

        # ── 1단계: 전처리 ─────────────────────────────────────────
        points = self.process.preprocess(msg)
        if points is None or len(points) < 10:
            # self.get_logger().warn('Preprocess: insufficient points, skipping frame')
            return

        times.append(time.time())

        self.pub_preprocessed.publish(numpy_to_ros(points, msg.header))
        # self.get_logger().info(f'[1] Preprocess: {len(points)} pts')

        # ── 2단계: Voxel 다운샘플링 ──────────────────────────────
        ds_points = self.process.downsample_voxel(points)
        times.append(time.time())
        if len(ds_points) == 0:
            return
        else:
            self.pub_downsampled.publish(numpy_to_ros(ds_points, msg.header))
        # self.get_logger().info(f'[2] Downsample: {len(ds_points)} pts')

        # ── 3단계: 바닥 제거 ──────────────────────────────────────
        # no_ground = self.process.remove_ground(ds_points)
        # self.get_logger().info(f'[3] Ground removed: {len(ds_points) - len(no_ground)} pts')
        # if len(no_ground) > 0:
        #     self.pub_no_ground.publish(numpy_to_ros(no_ground, msg.header))

        # ── 4단계: DBSCAN 클러스터링 ──────────────────────────────
        # labels = self.process.cluster_o3d(ds_points)
        # n_clusters = int((labels >= 0).any() and labels.max() + 1) if len(labels) > 0 else 0
        # self.get_logger().info(f'[4] Clusters: {n_clusters}')

        # if n_clusters == 0:
        #     status = String()
        #     status.data = 'SAFE: no objects detected'
        #     self.pub_collision.publish(status)
        #     return

        # # ── 5단계: AABB 계산 ──────────────────────────────────────
        # bboxes = self.process.compute_aabb(ds_points, labels)
        # self.get_logger().info(f'[5] AABB: {len(bboxes)} boxes')

        # # ── 6단계: Collision 판정 + 시각화 ───────────────────────
        # self.process.collision_and_visualize(bboxes, msg.header)


        spend_time = [times[i+1] - times[i] for i in range(len(times)-1)]
        spend_time = [f'{i*1000:.1f}ms' for i in spend_time]
        spend_time = ' | '.join(spend_time)
        elapsed = (times[-1] - times[0]) * 1000
        self.get_logger().info(
            f'Frame {self.frame_count:04d} | raw={len(points)} '
            # f'ds={len(ds_points)} clusters={len(bboxes)} | {elapsed:.1f}ms'
            f'ds={len(ds_points)} | total={elapsed:.1f}ms '
            f'\nEACH =  {spend_time}'
        )


# ============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepresentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
