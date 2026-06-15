#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kaair_msgs.srv import CreateObjectMarker


class ObjectMarkerServer(Node):
    def __init__(self):
        super().__init__('object_marker_server')
        self.map_frame = 'slamware_map'

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.latest_depth_msg = None
        self.latest_camera_info = None
        self.saved_objects = {}
        self.pending_delete_markers = []
        self.delete_republish_ticks = 0
        self.log_flags = {}

        # subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/femto/depth/image_raw',
            self.depth_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/femto/depth/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        # service server
        self.srv = self.create_service(
            CreateObjectMarker,
            'create_object_marker',
            self.handle_create_object_marker
        )
        self.clear_srv = self.create_service(
            Trigger,
            'clear_object_markers',
            self.handle_clear_object_markers
        )

        # tf listener for map transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/object_markers',
            10
        )

        # keep publishing saved markers
        self.publish_timer = self.create_timer(
            0.1, self.publish_saved_markers
        )

        self.get_logger().info('Object marker server started. v260615')

    def log_once(self, key, message, level='info'):
        if self.log_flags.get(key, False):
            return

        if level == 'warn':
            self.get_logger().warn(message)
        elif level == 'error':
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

        self.log_flags[key] = True

    def depth_callback(self, msg):
        self.latest_depth_msg = msg
        self.log_once(
            'depth_received',
            f'First depth received: width={msg.width}, height={msg.height}, '
            f'encoding={msg.encoding}, step={msg.step}, frame_id={msg.header.frame_id}'
        )

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        self.log_once(
            'camera_info_received',
            f'First camera_info received: fx={fx:.3f}, fy={fy:.3f}, '
            f'cx={cx:.3f}, cy={cy:.3f}, frame_id={msg.header.frame_id}'
        )

    def handle_create_object_marker(self, request, response):
        try:
            return self._handle_create_object_marker_impl(request, response)
        except Exception as e:
            self.get_logger().error(f'Unhandled exception in handle_create_object_marker: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f'internal_error: {e}'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            return response

    def _handle_create_object_marker_impl(self, request, response):
        object_label = request.object_label.strip()
        if object_label == '':
            object_label = 'object'
        object_name = f'/object/{object_label}'

        u_array = list(request.u_array)
        v_array = list(request.v_array)

        if len(u_array) == 0:
            response.success = False
            response.message = 'u_array and v_array must not be empty'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            return response

        if len(u_array) != len(v_array):
            response.success = False
            response.message = 'u_array and v_array must have the same length'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            return response

        base_frame = request.base_frame.strip()
        if base_frame == '':
            base_frame = self.map_frame

        map_points = []
        x_out = []
        y_out = []
        z_out = []
        no_depth_count = 0
        no_transform_count = 0

        for u, v in zip(u_array, v_array):
            point = self.get_3d_point_from_depth_pixel(int(u), int(v))
            if point is None:
                no_depth_count += 1
                x_out.append(float('nan'))
                y_out.append(float('nan'))
                z_out.append(float('nan'))
                continue

            map_x, map_y, map_z = point

            base_xyz = self.transform_point(map_x, map_y, map_z, self.map_frame, base_frame)
            if base_xyz is None:
                no_transform_count += 1
                x_out.append(float('nan'))
                y_out.append(float('nan'))
                z_out.append(float('nan'))
                continue

            map_points.append((map_x, map_y, map_z))
            bx, by, bz = base_xyz
            x_out.append(float(bx))
            y_out.append(float(by))
            z_out.append(float(bz))

        if map_points:
            self.saved_objects[object_name] = map_points
            self.get_logger().info(
                f'SUCCESS: "{object_name}" detected at {len(map_points)} pixel(s) '
                f'(skipped: no_depth={no_depth_count}, no_transform={no_transform_count}), '
                f'first map=({map_points[0][0]:.3f}, {map_points[0][1]:.3f}, {map_points[0][2]:.3f})'
            )
        else:
            self.get_logger().warn(
                f'"{object_name}": all {len(u_array)} pixel(s) have no valid depth '
                f'(no_depth={no_depth_count}, no_transform={no_transform_count}), returning false'
            )
            response.success = False
            response.message = f'all_no_depth (no_depth={no_depth_count}, no_transform={no_transform_count})'
            response.x_array = x_out
            response.y_array = y_out
            response.z_array = z_out
            return response

        response.success = True
        response.message = 'success'
        response.x_array = x_out
        response.y_array = y_out
        response.z_array = z_out
        self.get_logger().info(
            f'Response built: success=True, array_len={len(x_out)}, '
            f'no_depth={no_depth_count}, no_transform={no_transform_count}'
        )
        return response

    def get_3d_point_from_depth_pixel(self, u, v):
        if self.latest_depth_msg is None:
            self.log_once('no_depth', 'No depth image received yet.', level='warn')
            return None

        if self.latest_camera_info is None:
            self.log_once('no_camera_info', 'No camera_info received yet.', level='warn')
            return None

        depth_msg = self.latest_depth_msg
        cam_info = self.latest_camera_info

        if not (0 <= u < depth_msg.width and 0 <= v < depth_msg.height):
            self.get_logger().warn(
                f'Pixel out of depth range: ({u}, {v}), '
                f'size=({depth_msg.width}, {depth_msg.height})'
            )
            return None

        z = self.read_depth_value(depth_msg, u, v)
        if z is None or not math.isfinite(z) or z <= 0.0:
            z = self.search_valid_depth(depth_msg, u, v, radius=3)

        if z is None or not math.isfinite(z) or z <= 0.0:
            self.get_logger().warn(f'No valid depth near pixel ({u}, {v})')
            return None

        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        source_frame = depth_msg.header.frame_id
        map_point = self.transform_point(x, y, z, source_frame, self.map_frame)
        if map_point is None:
            return None

        map_x, map_y, map_z = map_point
        return (map_x, map_y, map_z)

    def handle_clear_object_markers(self, _request, response):
        object_names = [
            object_name for object_name in self.saved_objects.keys()
            if object_name.startswith('/object/')
        ]

        delete_array = MarkerArray()
        for object_name in object_names:
            points = self.saved_objects.get(object_name, [])
            delete_array.markers.append(self.build_delete_marker(object_name, marker_id=0))
            if len(points) == 1:
                delete_array.markers.append(self.build_delete_marker(object_name, marker_id=1))

        if delete_array.markers:
            self.marker_pub.publish(delete_array)
            # Republish delete markers briefly for reliable RViz cleanup.
            self.pending_delete_markers = list(delete_array.markers)
            self.delete_republish_ticks = 10

        for object_name in object_names:
            del self.saved_objects[object_name]

        self.get_logger().info(
            f'Cleared {len(object_names)} markers under /object namespace.'
        )
        response.success = True
        response.message = f'cleared_count={len(object_names)}'
        return response

    def transform_point(self, x, y, z, source_frame, target_frame):
        if source_frame == target_frame:
            return (x, y, z)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3)
            )
        except Exception as ex:
            self.get_logger().warn(
                f'Failed TF lookup {target_frame} <- {source_frame}: {ex}'
            )
            return None

        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w

        rx, ry, rz = self.rotate_vector_by_quaternion(x, y, z, qx, qy, qz, qw)
        return (tx + rx, ty + ry, tz + rz)

    def rotate_vector_by_quaternion(self, x, y, z, qx, qy, qz, qw):
        vx, vy, vz = x, y, z

        # t = 2 * (q_vec x v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        # v' = v + w*t + (q_vec x t)
        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return (rx, ry, rz)

    def read_depth_value(self, depth_msg, u, v):
        try:
            offset = v * depth_msg.step

            if depth_msg.encoding == '16UC1':
                byte_offset = offset + (u * 2)
                depth_raw = struct.unpack_from('<H', depth_msg.data, byte_offset)[0]
                if depth_raw == 0:
                    return None
                return depth_raw / 1000.0

            if depth_msg.encoding == '32FC1':
                byte_offset = offset + (u * 4)
                depth = struct.unpack_from('<f', depth_msg.data, byte_offset)[0]
                if not math.isfinite(depth) or depth <= 0.0:
                    return None
                return depth

            self.get_logger().error(
                f'Unsupported depth encoding: {depth_msg.encoding}'
            )
            return None

        except Exception as e:
            self.get_logger().error(f'Failed reading depth: {e}')
            return None

    def search_valid_depth(self, depth_msg, u, v, radius=3):
        for r in range(1, radius + 1):
            for dv in range(-r, r + 1):
                for du in range(-r, r + 1):
                    uu = u + du
                    vv = v + dv

                    if uu < 0 or uu >= depth_msg.width:
                        continue
                    if vv < 0 or vv >= depth_msg.height:
                        continue

                    z = self.read_depth_value(depth_msg, uu, vv)
                    if z is not None and math.isfinite(z) and z > 0.0:
                        self.log_once(
                            'nearby_depth_used',
                            'Used nearby valid depth near detected object.'
                        )
                        return z
        return None

    def build_arrow_marker(self, object_name, x, y, z):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = object_name
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = float(x)
        start.y = float(y)
        start.z = float(z)

        end = Point()
        end.x = float(x)
        end.y = float(y)
        end.z = float(z + 0.35)

        marker.points = [start, end]
        marker.scale.x = 0.03
        marker.scale.y = 0.06
        marker.scale.z = 0.1
        marker.lifetime = Duration(seconds=0.6).to_msg()

        marker.color.a = 1.0
        marker.color.r = 0.7
        marker.color.g = 0.2
        marker.color.b = 0.9

        return marker

    def build_text_marker(self, object_name, x, y, z):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = object_name
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z + 0.45)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.15
        marker.lifetime = Duration(seconds=0.6).to_msg()
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = object_name

        return marker

    def build_points_marker(self, object_name, points):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = object_name
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        for x, y, z in points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            marker.points.append(p)

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.lifetime = Duration(seconds=0.6).to_msg()

        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5

        return marker

    def build_delete_marker(self, object_name, marker_id):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = object_name
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def publish_saved_markers(self):
        marker_array = MarkerArray()
        if self.delete_republish_ticks > 0 and self.pending_delete_markers:
            marker_array.markers.extend(self.pending_delete_markers)
            self.delete_republish_ticks -= 1

        for object_name, points in self.saved_objects.items():
            if len(points) == 1:
                x, y, z = points[0]
                marker_array.markers.append(self.build_arrow_marker(object_name, x, y, z))
                marker_array.markers.append(self.build_text_marker(object_name, x, y, z))
            elif len(points) >= 2:
                marker_array.markers.append(self.build_points_marker(object_name, points))

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()