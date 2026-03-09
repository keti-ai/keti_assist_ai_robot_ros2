import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject

import numpy as np

from kaair_obstacle.utils.load_config import load_config, PipelineConfig
from kaair_obstacle.utils.convert_pointcloud import numpy_to_ros
from kaair_obstacle.utils.process import Process


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
        self.pub_preprocessed    = self.create_publisher(PointCloud2,     '/pointcloud/preprocessed', 10)
        self.pub_downsampled     = self.create_publisher(PointCloud2,     '/pointcloud/downsampled',  10)
        # self.pub_mesh            = self.create_publisher(MarkerArray,     '/pointcloud/mesh',          10)
        self.pub_shape_markers   = self.create_publisher(MarkerArray,     '/obstacle_spheres',         10)
        self.pub_collision_obj   = self.create_publisher(CollisionObject, '/collision_field',         10)

        self.frame_count = 0
        self.get_logger().info(
            f'ObstacleRepresentation started | '
            f'topic={self.cfg.input_topic} | voxel={self.cfg.voxel_size}m | marker_shape={self.cfg.marker_shape}'
        )

    # ----------------------------------------------------------------
    def _bind_cfg(self):
        """PipelineConfig 값을 self 속성으로 단축 연결"""
        c = self.cfg
        self.voxel_size          = c.voxel_size
        self.marker_shape        = c.marker_shape
        self.roi_x_min           = c.roi_x_min
        self.roi_x_max           = c.roi_x_max
        self.roi_y_min           = c.roi_y_min
        self.roi_y_max           = c.roi_y_max
        self.roi_z_min           = c.roi_z_min
        self.roi_z_max           = c.roi_z_max

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

        # ── 2.5단계: 메시 생성 + 퍼블리시 ───────────────────────
        # mesh_mk = self.process.create_mesh_marker(ds_points, msg.header)
        # times.append(time.time())
        # if mesh_mk is not None:
        #     ma = MarkerArray()
        #     ma.markers.append(mesh_mk)
        #     self.pub_mesh.publish(ma)

        shape_mk = self.process.create_shape_markers(
            ds_points, 
            self.marker_shape, 
            msg.header.frame_id, 
            msg.header.stamp)
        times.append(time.time())
        self.pub_shape_markers.publish(shape_mk)

        # MoveIt2 CollisionObject 생성 및 퍼블리시
        collision_obj = self.process.create_collision_object(
            ds_points, 
            msg.header.frame_id, 
            msg.header.stamp, 
            shape=self.marker_shape
        )
        times.append(time.time())
        self.pub_collision_obj.publish(collision_obj)

        spend_time = [times[i+1] - times[i] for i in range(len(times)-1)]
        spend_time = [f'{i*1000:.1f}ms' for i in spend_time]
        spend_time = ' | '.join(spend_time)
        elapsed = (times[-1] - times[0]) * 1000
        self.get_logger().info(
            f'\nFrame {self.frame_count:04d} | raw={len(points)} | ds={len(ds_points)} '
            # f'ds={len(ds_points)} clusters={len(bboxes)} | {elapsed:.1f}ms'
            f'\nTotal={elapsed:.1f}ms | EACH =  {spend_time}'
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
