#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker

# 🌟 QoS 설정을 위한 라이브러리 추가
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# TF 및 PointCloud2 처리를 위한 라이브러리 import
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs # PoseStamped 변환을 위해 필수!
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class VisionHelperNode(Node):
    def __init__(self):
        super().__init__('vision_helper_node')

        # --- [A] 파라미터 설정 (필요시 수정) ---
        # 1. 입력 토픽 이름
        self.pc_topic = '/femto/depth_registered/points'  # 사용하는 카메라 토픽으로 변경
        self.camera_frame = 'femto_color_optical_frame' # 카메라의 Optical TF 프레임

        # 2. 출력(최종 목표) 좌표계 설정
        self.base_frame = 'base_footprint' # 로봇의 중심 좌표계

        # --- [B] TF Listener 초기화 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- [C] 상태 변수 및 Subscriber 설정 ---
        self.latest_cloud = None
        
        # 🌟 커스텀 QoS 프로필 생성: BEST_EFFORT, Depth=1 (네트워크 부하 최소화)
        pc_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 수정한 QoS 프로필을 적용하여 구독자 생성
        self.pc_sub = self.create_subscription(
            PointCloud2, 
            self.pc_topic, 
            self.cloud_callback, 
            pc_qos_profile
        )
        
        # 외부에서 목표 픽셀(u, v)을 넘겨받는 토픽 (x=u, y=v 사용)
        self.pixel_sub = self.create_subscription(
            Point, '/target_pixel_2d', self.pixel_callback, 10)

        # --- [D] Publisher 설정 ---
        # 1. MoveIt용: base_footprint 기준의 PoseStamped
        self.pose_pub = self.create_publisher(
            PoseStamped, '/manipulator_target_pose', 10)
        
        # 2. RViz2 확인용: base_footprint 기준의 Marker
        self.marker_pub = self.create_publisher(
            Marker, '/vision_target_marker', 10)

        self.get_logger().info(f'👁️ Vision Helper 노드 시작. ({self.camera_frame} -> {self.base_frame})')

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def pixel_callback(self, msg):
        if self.latest_cloud is None:
            self.get_logger().warn('⚠️ 아직 PointCloud 데이터를 받지 못했습니다.')
            return

        u = int(msg.x)
        v = int(msg.y)
        
        # 🌟 수정한 부분: Unordered(1D) 클라우드 대응
        # 만약 height가 1이라면, 원본 해상도(640x480 등)를 직접 지정해줘야 합니다.
        if self.latest_cloud.height == 1:
            # 640x480 해상도라고 가정 (카메라 설정값과 맞춰주세요)
            target_width = 640 
            target_height = 480
            
            if u < 0 or u >= target_width or v < 0 or v >= target_height:
                self.get_logger().error(f'❌ 픽셀 ({u}, {v})가 범위를 벗어났습니다. (Target: {target_width}x{target_height})')
                return
            
            # 1D 배열에서의 인덱스 계산
            idx = v * target_width + u
            uvs = [idx] # read_points에서 인덱스로 접근
        else:
            # Ordered(2D) 클라우드인 경우 기존 방식 사용
            if u < 0 or u >= self.latest_cloud.width or v < 0 or v >= self.latest_cloud.height:
                self.get_logger().error(f'❌ 픽셀 ({u}, {v})가 범위를 벗어났습니다. (Max: {self.latest_cloud.width}x{self.latest_cloud.height})')
                return
            uvs = [(u, v)]

        # XYZ 추출
        gen = pc2.read_points(self.latest_cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=uvs)
        
        try:
            target_xyz = next(gen)
            x_cam, y_cam, z_cam = target_xyz[0], target_xyz[1], target_xyz[2]

            # 2차 예외 처리: Depth 데이터 유효성 확인
            import math
            if math.isnan(x_cam) or math.isnan(y_cam) or math.isnan(z_cam):
                self.get_logger().warn(f'⚠️ 픽셀 ({u}, {v}) 위치에 Depth 데이터가 없습니다.')
                return

            self.get_logger().info(f'🎯 픽셀({u}, {v}) -> Camera 좌표: X={x_cam:.3f}, Y={y_cam:.3f}, Z={z_cam:.3f}')
            
            # 🌟 핵심 2: TF 변환 함수 호출!
            self.transform_and_publish(x_cam, y_cam, z_cam)

        except StopIteration:
            self.get_logger().error('❌ 3D 포인트를 추출하는 데 실패했습니다.')

    def transform_and_publish(self, x, y, z):
        """camera_frame -> base_frame 으로 XYZ를 변환하고 퍼블리시"""
        
        now = self.get_clock().now().to_msg()

        # 1. 변환할 데이터를 PoseStamped에 담습니다 (Camera Frame 기준)
        pose_cam = PoseStamped()
        pose_cam.header.stamp = now # 중요! PointCloud를 받은 시간과 맞추는 것이 정석이지만, 단순화를 위해 현재 시간 사용
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = float(x)
        pose_cam.pose.position.y = float(y)
        pose_cam.pose.position.z = float(z)
        pose_cam.pose.orientation.w = 1.0 # 기본 방향

        try:
            # 🌟 핵심 3: TF Buffer에서 camera_frame -> base_frame 변환 요청
            # tf_buffer.transform() 함수는 데이터를 자동으로 변환해서 PoseStamped로 반환해 줍니다.
            pose_base = self.tf_buffer.transform(pose_cam, self.base_frame)

            self.get_logger().info(
                f'🦾 Base 좌표 변환 완료: X={pose_base.pose.position.x:.3f}, '
                f'Y={pose_base.pose.position.y:.3f}, Z={pose_base.pose.position.z:.3f}'
            )

            # [A] 퍼블리시: base_footprint 기준 PoseStamped (MoveIt용)
            self.pose_pub.publish(pose_base)

            # [B] 퍼블리시: base_footprint 기준 Marker (RViz용)
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.base_frame # 중요! 변환된 좌표계 이름
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pose_base.pose.position # 변환된 XYZ 사용
            
            # 크기 (5cm) 및 색상 (빨간색)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.a = 0.8 # 투명도
            self.marker_pub.publish(marker)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'❌ TF 변환 실패 ({self.camera_frame} -> {self.base_frame}): {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionHelperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()