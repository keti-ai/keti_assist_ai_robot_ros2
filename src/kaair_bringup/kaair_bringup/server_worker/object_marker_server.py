#!/usr/bin/env python3

import math
import struct
import traceback

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
        self.latest_color_msg = None
        self.latest_camera_info = None
        self.latest_tool_depth_msg = None
        self.latest_tool_camera_info = None
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

        self.color_sub = self.create_subscription(
            Image,
            '/femto/color/image_raw',
            self.color_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/femto/depth/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        self.tool_depth_sub = self.create_subscription(
            Image,
            '/hand/d405/depth/image_rect_raw',
            self.tool_depth_callback,
            sensor_qos
        )
        self.tool_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/hand/d405/depth/camera_info',
            self.tool_camera_info_callback,
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
        self.tool_srv = self.create_service(
            CreateObjectMarker,
            'create_tool_marker',
            self.handle_create_tool_marker
        )
        self.clear_tool_srv = self.create_service(
            Trigger,
            'clear_tool_markers',
            self.handle_clear_tool_markers
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

        self.get_logger().info('Object marker server started (femto + d405 tool).')

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

    def color_callback(self, msg):
        self.latest_color_msg = msg
        self.log_once(
            'color_received',
            f'First color received: width={msg.width}, height={msg.height}, '
            f'encoding={msg.encoding}, frame_id={msg.header.frame_id}'
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

    def tool_depth_callback(self, msg):
        self.latest_tool_depth_msg = msg
        self.log_once(
            'tool_depth_received',
            f'First tool depth received: width={msg.width}, height={msg.height}, '
            f'encoding={msg.encoding}, step={msg.step}, frame_id={msg.header.frame_id}'
        )

    def tool_camera_info_callback(self, msg):
        self.latest_tool_camera_info = msg
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        self.log_once(
            'tool_camera_info_received',
            f'First tool camera_info received: fx={fx:.3f}, fy={fy:.3f}, '
            f'cx={cx:.3f}, cy={cy:.3f}, frame_id={msg.header.frame_id}'
        )

    # ──────────────────────────────────────────────
    # Service handler (exception wrapper)
    # ──────────────────────────────────────────────

    def handle_create_object_marker(self, request, response):
        try:
            return self._handle_create_object_marker_impl(request, response)
        except Exception as e:
            self.get_logger().error(f'Unhandled exception in handle_create_object_marker: {e}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f'internal_error: {e}'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            response.depth_raw_array = []
            response.r_array = []
            response.g_array = []
            response.b_array = []
            return response

    def _handle_create_object_marker_impl(self, request, response):
        object_label = request.object_label.strip()
        if object_label == '':
            object_label = 'object'
        object_name = f'/object/{object_label}'
        return self._process_marker_request(
            request, response, object_name,
            self.latest_depth_msg, self.latest_camera_info
        )

    def _process_marker_request(self, request, response, object_name, depth_msg, camera_info):
        u_array = list(request.u_array)
        v_array = list(request.v_array)
        d_array = list(request.d_array)  # optional; empty = use depth image for all

        if len(u_array) == 0:
            response.success = False
            response.message = 'u_array and v_array must not be empty'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            response.depth_raw_array = []
            response.r_array = []
            response.g_array = []
            response.b_array = []
            return response

        if len(u_array) != len(v_array):
            response.success = False
            response.message = 'u_array and v_array must have the same length'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            response.depth_raw_array = []
            response.r_array = []
            response.g_array = []
            response.b_array = []
            return response

        # d_array: must be empty OR same length as u_array
        if len(d_array) > 0 and len(d_array) != len(u_array):
            self.get_logger().warn(
                f'd_array length ({len(d_array)}) != u_array length ({len(u_array)}), ignoring d_array'
            )
            d_array = []

        base_frame = request.base_frame.strip()
        if base_frame == '':
            base_frame = self.map_frame

        map_points = []
        x_out = []
        y_out = []
        z_out = []
        depth_raw_out = []
        r_out = []
        g_out = []
        b_out = []
        no_depth_count = 0
        no_transform_count = 0

        for i, (u, v) in enumerate(zip(u_array, v_array)):
            u, v = int(u), int(v)

            # ── depth_raw: always read actual sensor pixel value (no neighbor search) ──
            depth_raw = self._read_depth_raw(u, v, depth_msg=depth_msg)
            depth_raw_out.append(depth_raw if depth_raw is not None else 0.0)

            # ── RGB at (u, v) ──
            r, g, b = self._read_color_pixel(u, v)
            r_out.append(r)
            g_out.append(g)
            b_out.append(b)

            # ── Determine z for 3D projection ──
            depth_override = None
            if d_array:
                d = d_array[i]
                if math.isfinite(d) and d > 0.0:
                    depth_override = d

            if depth_override is not None:
                z = depth_override
            else:
                z = self._get_depth_z(u, v, depth_msg=depth_msg)
                if z is None:
                    no_depth_count += 1
                    x_out.append(float('nan'))
                    y_out.append(float('nan'))
                    z_out.append(float('nan'))
                    continue

            # ── Project pixel + z → map frame ──
            map_point = self._project_to_map(u, v, z, depth_msg=depth_msg, camera_info=camera_info)
            if map_point is None:
                no_transform_count += 1
                x_out.append(float('nan'))
                y_out.append(float('nan'))
                z_out.append(float('nan'))
                continue

            map_x, map_y, map_z = map_point

            # ── Transform map → base_frame ──
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
            response.depth_raw_array = depth_raw_out
            response.r_array = r_out
            response.g_array = g_out
            response.b_array = b_out
            return response

        response.success = True
        response.message = 'success'
        response.x_array = x_out
        response.y_array = y_out
        response.z_array = z_out
        response.depth_raw_array = depth_raw_out
        response.r_array = r_out
        response.g_array = g_out
        response.b_array = b_out
        return response

    # ──────────────────────────────────────────────
    # Depth helpers
    # ──────────────────────────────────────────────

    def _read_depth_raw(self, u, v, depth_msg=None):
        """Return raw depth (metres) at pixel (u,v) with NO neighbour search. None if unavailable."""
        dm = depth_msg if depth_msg is not None else self.latest_depth_msg
        if dm is None:
            return None
        if not (0 <= u < dm.width and 0 <= v < dm.height):
            return None
        return self.read_depth_value(dm, u, v)

    def _get_depth_z(self, u, v, depth_msg=None):
        """Return depth z (metres) from depth image, with neighbour fallback. None if unavailable."""
        dm = depth_msg if depth_msg is not None else self.latest_depth_msg
        if dm is None:
            self.log_once('no_depth', 'No depth image received yet.', level='warn')
            return None
        if not (0 <= u < dm.width and 0 <= v < dm.height):
            self.get_logger().warn(
                f'Pixel out of depth range: ({u}, {v}), size=({dm.width}, {dm.height})'
            )
            return None

        z = self.read_depth_value(dm, u, v)
        if z is None or not math.isfinite(z) or z <= 0.0:
            z = self.search_valid_depth(dm, u, v, radius=3)

        if z is None or not math.isfinite(z) or z <= 0.0:
            self.get_logger().warn(f'No valid depth near pixel ({u}, {v})')
            return None

        return z

    def _project_to_map(self, u, v, z, depth_msg=None, camera_info=None):
        """Project pixel (u,v) with depth z → map frame. Returns (x,y,z) or None."""
        cam = camera_info if camera_info is not None else self.latest_camera_info
        dm = depth_msg if depth_msg is not None else self.latest_depth_msg

        if cam is None:
            self.log_once('no_camera_info', 'No camera_info received yet.', level='warn')
            return None

        fx, fy = cam.k[0], cam.k[4]
        cx, cy = cam.k[2], cam.k[5]

        px = (u - cx) * z / fx
        py = (v - cy) * z / fy

        if dm is not None:
            source_frame = dm.header.frame_id
        else:
            source_frame = cam.header.frame_id

        return self.transform_point(px, py, z, source_frame, self.map_frame)

    # ──────────────────────────────────────────────
    # Color helper
    # ──────────────────────────────────────────────

    def _read_color_pixel(self, u, v):
        """Return (r, g, b) uint8 at pixel (u,v). Returns (0, 0, 0) if unavailable."""
        if self.latest_color_msg is None:
            return 0, 0, 0

        msg = self.latest_color_msg

        # Scale u,v if color image has different resolution than depth
        if self.latest_depth_msg is not None and (
            msg.width != self.latest_depth_msg.width or
            msg.height != self.latest_depth_msg.height
        ):
            uc = int(u * msg.width / self.latest_depth_msg.width)
            vc = int(v * msg.height / self.latest_depth_msg.height)
        else:
            uc, vc = u, v

        if not (0 <= uc < msg.width and 0 <= vc < msg.height):
            return 0, 0, 0

        try:
            enc = msg.encoding.lower()
            offset = vc * msg.step

            if enc in ('rgb8', 'rgb'):
                base = offset + uc * 3
                r = msg.data[base]
                g = msg.data[base + 1]
                b = msg.data[base + 2]

            elif enc in ('bgr8', 'bgr'):
                base = offset + uc * 3
                b = msg.data[base]
                g = msg.data[base + 1]
                r = msg.data[base + 2]

            elif enc in ('rgba8', 'rgba'):
                base = offset + uc * 4
                r = msg.data[base]
                g = msg.data[base + 1]
                b = msg.data[base + 2]

            elif enc in ('bgra8', 'bgra'):
                base = offset + uc * 4
                b = msg.data[base]
                g = msg.data[base + 1]
                r = msg.data[base + 2]

            elif enc in ('mono8', '8uc1'):
                val = msg.data[offset + uc]
                r, g, b = val, val, val

            else:
                self.log_once(
                    f'unsupported_color_enc_{enc}',
                    f'Unsupported color encoding: {msg.encoding}',
                    level='warn'
                )
                return 0, 0, 0

            return int(r), int(g), int(b)

        except Exception as e:
            self.get_logger().error(f'Failed reading color pixel ({u},{v}): {e}')
            return 0, 0, 0

    # ──────────────────────────────────────────────
    # Legacy 3D point method (kept for compatibility)
    # ──────────────────────────────────────────────

    def get_3d_point_from_depth_pixel(self, u, v):
        z = self._get_depth_z(u, v)
        if z is None:
            return None
        return self._project_to_map(u, v, z)

    # ──────────────────────────────────────────────
    # Tool marker service handlers
    # ──────────────────────────────────────────────

    def handle_create_tool_marker(self, request, response):
        try:
            return self._handle_create_tool_marker_impl(request, response)
        except Exception as e:
            self.get_logger().error(f'Unhandled exception in handle_create_tool_marker: {e}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f'internal_error: {e}'
            response.x_array = []
            response.y_array = []
            response.z_array = []
            response.depth_raw_array = []
            response.r_array = []
            response.g_array = []
            response.b_array = []
            return response

    def _handle_create_tool_marker_impl(self, request, response):
        object_label = request.object_label.strip()
        if object_label == '':
            object_label = 'tool'
        object_name = f'/tool/{object_label}'
        return self._process_marker_request(
            request, response, object_name,
            self.latest_tool_depth_msg, self.latest_tool_camera_info
        )

    # ──────────────────────────────────────────────
    # Clear markers
    # ──────────────────────────────────────────────

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

    def handle_clear_tool_markers(self, _request, response):
        tool_names = [
            object_name for object_name in self.saved_objects.keys()
            if object_name.startswith('/tool/')
        ]

        delete_array = MarkerArray()
        for object_name in tool_names:
            points = self.saved_objects.get(object_name, [])
            delete_array.markers.append(self.build_delete_marker(object_name, marker_id=0))
            if len(points) == 1:
                delete_array.markers.append(self.build_delete_marker(object_name, marker_id=1))

        if delete_array.markers:
            self.marker_pub.publish(delete_array)
            self.pending_delete_markers = list(delete_array.markers)
            self.delete_republish_ticks = 10

        for object_name in tool_names:
            del self.saved_objects[object_name]

        self.get_logger().info(
            f'Cleared {len(tool_names)} markers under /tool namespace.'
        )
        response.success = True
        response.message = f'cleared_count={len(tool_names)}'
        return response

    # ──────────────────────────────────────────────
    # TF helpers
    # ──────────────────────────────────────────────

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

    # ──────────────────────────────────────────────
    # Depth image reading
    # ──────────────────────────────────────────────

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

    # ──────────────────────────────────────────────
    # Marker builders
    # ──────────────────────────────────────────────

    def build_arrow_marker(self, object_name, x, y, z, is_tool=False):
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
        if is_tool:
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0
        else:
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

    def build_points_marker(self, object_name, points, is_tool=False):
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
        if is_tool:
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0
        else:
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
            is_tool = object_name.startswith('/tool/')
            if len(points) == 1:
                x, y, z = points[0]
                marker_array.markers.append(self.build_arrow_marker(object_name, x, y, z, is_tool=is_tool))
                marker_array.markers.append(self.build_text_marker(object_name, x, y, z))
            elif len(points) >= 2:
                marker_array.markers.append(self.build_points_marker(object_name, points, is_tool=is_tool))

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
