import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

import tf2_ros

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import  MarkerArray
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

        # ── TF2 (좌표계 변환) ─────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_footprint'
        ).get_parameter_value().string_value

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
        self.pub_shape_markers   = self.create_publisher(MarkerArray,     '/scene/obstacle_markers',         10)
        self.pub_collision_obj   = self.create_publisher(CollisionObject, '/scene/collision_objects',         10)

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
    # TF 변환
    # ================================================================
    def _transform_points(
        self,
        points: np.ndarray,
        source_frame: str,
        stamp,
    ) -> tuple:
        """
        (N,3) float32 포인트를 source_frame → target_frame으로 변환.

        TF 조회 실패 시 원본 포인트와 source_frame을 그대로 반환하여
        파이프라인이 중단되지 않도록 한다.

        Returns:
            (transformed_points, actual_frame_id)
        """
        if source_frame == self.target_frame:
            return points, source_frame

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,      # 0 = 최신 TF 사용 (타임스탬프 불일치 방지)
                timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {source_frame} → {self.target_frame} 실패: {e}',
                throttle_duration_sec=2.0,
            )
            return points, source_frame

        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w

        # 쿼터니언 → 3×3 회전행렬 (numpy)
        R = np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)  ],
            [2*(x*y + w*z),       1 - 2*(x*x + z*z),  2*(y*z - w*x)  ],
            [2*(x*z - w*y),       2*(y*z + w*x),      1 - 2*(x*x + y*y)],
        ], dtype=np.float32)
        T = np.array([t.x, t.y, t.z], dtype=np.float32)

        transformed = (R @ points.T).T + T
        return transformed.astype(np.float32), self.target_frame

    # ================================================================
    # 메인 콜백
    # ================================================================
    def callback(self, msg: PointCloud2):
        # times = [time.time()]
        # self.frame_count += 1

        # ── 전처리 (카메라 프레임, ROI 필터 포함) ──────────
        points = self.process.preprocess(msg)
        if points is None or len(points) < 10:
            return

        # times.append(time.time())

        # ── Voxel 다운샘플링 ──────────────────────────────
        ds_points = self.process.downsample_voxel(points)
        # times.append(time.time())

        # ── TF 변환 (카메라 프레임 → target_frame) ──────
        tf_points, frame_id = self._transform_points(ds_points, msg.header.frame_id, msg.header.stamp)
        # times.append(time.time())

        # MoveIt2 CollisionObject 생성 및 퍼블리시
        collision_obj = self.process.create_collision_object(
            tf_points,
            frame_id,
            msg.header.stamp,
            shape=self.marker_shape
        )
        # times.append(time.time())
        self.pub_collision_obj.publish(collision_obj)

        # rviz2용 시각화 -> voxel사이즈 더 큼
        shape_mk = self.process.create_shape_markers(
            tf_points,
            self.marker_shape,
            frame_id,
            msg.header.stamp)
        # times.append(time.time())
        self.pub_shape_markers.publish(shape_mk)

        # times.append(time.time())

        # spend_time = [times[i+1] - times[i] for i in range(len(times)-1)]
        # spend_time = [f'{i*1000:.1f}ms' for i in spend_time]
        # spend_time = ' | '.join(spend_time)
        # elapsed = (times[-1] - times[0]) * 1000
        # self.get_logger().info(
        #     f'\nFrame {self.frame_count:04d} | raw={len(points)} | ds={len(ds_points)} '
        #     # f'ds={len(ds_points)} clusters={len(bboxes)} | {elapsed:.1f}ms'
        #     f'\nTotal={elapsed:.1f}ms | EACH =  {spend_time}'
        # )

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
