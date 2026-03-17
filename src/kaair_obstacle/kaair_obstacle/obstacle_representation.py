import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

import tf2_ros

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import CollisionObject

import numpy as np

from kaair_obstacle.utils.load_config import load_config, PipelineConfig
from kaair_obstacle.utils.process import Process

import gc


class ObstacleRepresentation(Node):

    def __init__(self):
        super().__init__('obstacle_representation_node')

        gc.set_threshold(10000, 100, 100)

        # ── 설정 로드 ──────────────────────────────────────────────
        self.cfg: PipelineConfig = load_config(self)
        self._bind_cfg()

        # ── 파이프라인 처리 객체 ───────────────────────────────────
        self.process = Process(self)

        # ── 콜백 그룹 (TF / PCL 병렬 처리) ───────────────────────
        self._tf_cb_group  = MutuallyExclusiveCallbackGroup()
        self._pcl_cb_group = MutuallyExclusiveCallbackGroup()

        # ── TF2 ───────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self,
            spin_thread=False,  # executor가 관리
        )
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_footprint'
        ).get_parameter_value().string_value

        # ── TF 캐시 (lookup 실패 시 마지막 성공 TF 재사용) ────────
        self._latest_tf = None

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
            sensor_qos,
            callback_group=self._pcl_cb_group,
        )

        # ── Publishers ────────────────────────────────────────────
        self.pub_shape_markers = self.create_publisher(MarkerArray,     '/scene/obstacle_markers',  10)
        self.pub_collision_obj = self.create_publisher(CollisionObject, '/scene/collision_objects',  10)

        self.get_logger().info(
            f'ObstacleRepresentation started | '
            f'topic={self.cfg.input_topic} | voxel={self.cfg.voxel_size}m | marker_shape={self.cfg.marker_shape}'
        )

    # ----------------------------------------------------------------
    def _bind_cfg(self):
        c = self.cfg
        self.voxel_size   = c.voxel_size
        self.marker_shape = c.marker_shape
        self.roi_x_min    = c.roi_x_min
        self.roi_x_max    = c.roi_x_max
        self.roi_y_min    = c.roi_y_min
        self.roi_y_max    = c.roi_y_max
        self.roi_z_min    = c.roi_z_min
        self.roi_z_max    = c.roi_z_max

    # ================================================================
    # TF 변환
    # ================================================================
    def _transform_points(self, points: np.ndarray, tf) -> tuple:
        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2*(y*y + z*z),  2*(x*y - w*z),     2*(x*z + w*y)   ],
            [2*(x*y + w*z),      1 - 2*(x*x + z*z),  2*(y*z - w*x)   ],
            [2*(x*z - w*y),      2*(y*z + w*x),      1 - 2*(x*x + y*y)],
        ], dtype=np.float32)
        T = np.array([t.x, t.y, t.z], dtype=np.float32)

        transformed = (R @ points.T).T + T
        return transformed.astype(np.float32), self.target_frame

    # ================================================================
    # 메인 콜백
    # ================================================================
    def callback(self, msg: PointCloud2):

        # ── TF 조회 (실패 시 마지막 성공 TF 재사용) ───────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0),
            )
            self._latest_tf = tf
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if self._latest_tf is None:
                self.get_logger().warn(
                    f'TF 아직 수신 안됨, 스킵: {e}',
                    throttle_duration_sec=2.0
                )
                return
            tf = self._latest_tf

        # ── 전처리 (ROI 필터 포함) ────────────────────────────────
        points = self.process.preprocess(msg)
        if points is None or len(points) < 10:
            return

        # ── Voxel 다운샘플링 ──────────────────────────────────────
        ds_points = self.process.downsample_voxel(points)
        if len(ds_points) == 0:
            return

        # ── TF 변환 (카메라 프레임 → target_frame) ────────────────
        if msg.header.frame_id == self.target_frame:
            tf_points = ds_points
            frame_id  = msg.header.frame_id
        else:
            tf_points, frame_id = self._transform_points(ds_points, tf)

        # ── CollisionObject 생성 및 퍼블리시 ──────────────────────
        collision_obj = self.process.create_collision_object(
            tf_points, frame_id, msg.header.stamp, shape=self.marker_shape)
        self.pub_collision_obj.publish(collision_obj)

        # ── MarkerArray 생성 및 퍼블리시 ──────────────────────────
        shape_mk = self.process.create_shape_markers(
            tf_points, self.marker_shape, frame_id, msg.header.stamp)
        self.pub_shape_markers.publish(shape_mk)


# ============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepresentation()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()