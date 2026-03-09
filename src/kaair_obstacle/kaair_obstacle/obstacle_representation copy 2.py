import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d


class PointCloudToSpheres(Node):
    def __init__(self):
        super().__init__('pointcloud_to_spheres')

        # 파라미터
        self.declare_parameter('voxel_size', 0.05)      # 구 반지름 (미터)
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('min_points', 5)

        self.voxel_size = self.get_parameter('voxel_size').value

        # 구독: 포인트클라우드
        self.sub = self.create_subscription(
            PointCloud2,
            '/femto/depth_registered/points',   # 실제 토픽명으로 변경
            self.pointcloud_callback,
            10
        )

        # 퍼블리시: MarkerArray (RViz2 시각화용)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacle_spheres',
            10
        )

        self.get_logger().info('PointCloudToSpheres 노드 시작')


    def pointcloud_callback(self, msg: PointCloud2):
        # ── 1. PointCloud2 → numpy ──
        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float32)

        if len(points) < 10:
            return

        # ── 2. 포인트클라우드 → 구 집합 (voxel 다운샘플링) ──
        sphere_centers = self.pointcloud_to_spheres(points)

        # ── 3. MarkerArray 퍼블리시 ──
        marker_array = self.create_sphere_markers(
            sphere_centers,
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp,
        )
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'구 {len(sphere_centers)}개 퍼블리시')


    def pointcloud_to_spheres(self, points: np.ndarray) -> np.ndarray:
        """포인트클라우드 → 복셀 중심(구 중심) 리스트"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 노이즈 제거
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # 복셀 다운샘플링 → 각 복셀 중심 = 구 중심
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        return np.asarray(pcd_down.points)


    def create_sphere_markers(
        self,
        centers: np.ndarray,
        frame_id: str,
        stamp,
        color: tuple = (1.0, 0.3, 0.0, 0.7),  # RGBA
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
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # 구 크기
        # marker.scale.x = self.voxel_size * 2  # 지름 = 반지름 * 2
        # marker.scale.y = self.voxel_size * 2
        # marker.scale.z = self.voxel_size * 2
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size

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


def main():
    rclpy.init()
    node = PointCloudToSpheres()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()