"""
Decodes raw zenoh payloads into readable text for the Message tab, the same
way `ros2 topic echo` shows a message's fields instead of raw bytes. Only a
hand-picked set of common ROS2 message types is covered (whatever this
project's topics actually use) since there is no ROS2 install here to
introspect arbitrary message definitions from - unknown types fall back to
a raw best-effort text view in raw_message_manager.py.
"""

from .cdr import CDRReader


def _header(r):
    sec = r.i32()
    nanosec = r.u32()
    frame_id = r.string()
    return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame_id}


def _vector3(r):
    return {"x": r.f64(), "y": r.f64(), "z": r.f64()}


def _quaternion(r):
    return {"x": r.f64(), "y": r.f64(), "z": r.f64(), "w": r.f64()}


def _pose(r):
    return {"position": _vector3(r), "orientation": _quaternion(r)}


def _twist(r):
    return {"linear": _vector3(r), "angular": _vector3(r)}


def _fixed_f64_array(r, count):
    return [r.f64() for _ in range(count)]


def decode_joint_state(payload):
    r = CDRReader(payload)
    return {
        "header": _header(r),
        "name": r.string_sequence(),
        "position": r.f64_sequence(),
        "velocity": r.f64_sequence(),
        "effort": r.f64_sequence(),
    }


def decode_pose_stamped(payload):
    r = CDRReader(payload)
    return {"header": _header(r), "pose": _pose(r)}


def decode_odometry(payload):
    r = CDRReader(payload)
    header = _header(r)
    child_frame_id = r.string()
    pose = _pose(r)
    pose_covariance = _fixed_f64_array(r, 36)
    twist = _twist(r)
    twist_covariance = _fixed_f64_array(r, 36)
    return {
        "header": header,
        "child_frame_id": child_frame_id,
        "pose": {"pose": pose, "covariance": pose_covariance},
        "twist": {"twist": twist, "covariance": twist_covariance},
    }


def decode_imu(payload):
    r = CDRReader(payload)
    return {
        "header": _header(r),
        "orientation": _quaternion(r),
        "orientation_covariance": _fixed_f64_array(r, 9),
        "angular_velocity": _vector3(r),
        "angular_velocity_covariance": _fixed_f64_array(r, 9),
        "linear_acceleration": _vector3(r),
        "linear_acceleration_covariance": _fixed_f64_array(r, 9),
    }


def decode_image(payload):
    r = CDRReader(payload)
    header = _header(r)
    height = r.u32()
    width = r.u32()
    encoding = r.string()
    is_bigendian = bool(r.u8())
    step = r.u32()
    data = r.byte_array()
    return {
        "header": header,
        "height": height,
        "width": width,
        "encoding": encoding,
        "is_bigendian": is_bigendian,
        "step": step,
        "data": f"<{len(data)} bytes, omitted>",
    }


def decode_compressed_image(payload):
    r = CDRReader(payload)
    header = _header(r)
    fmt = r.string()
    data = r.byte_array()
    return {"header": header, "format": fmt, "data": f"<{len(data)} bytes, omitted>"}


DECODERS = {
    "std_msgs/msg/String": lambda p: {"data": CDRReader(p).string()},
    "std_msgs/msg/Bool": lambda p: {"data": CDRReader(p).bool()},
    "std_msgs/msg/Int8": lambda p: {"data": CDRReader(p).i8()},
    "std_msgs/msg/UInt8": lambda p: {"data": CDRReader(p).u8()},
    "std_msgs/msg/Int16": lambda p: {"data": CDRReader(p).i16()},
    "std_msgs/msg/UInt16": lambda p: {"data": CDRReader(p).u16()},
    "std_msgs/msg/Int32": lambda p: {"data": CDRReader(p).i32()},
    "std_msgs/msg/UInt32": lambda p: {"data": CDRReader(p).u32()},
    "std_msgs/msg/Int64": lambda p: {"data": CDRReader(p).i64()},
    "std_msgs/msg/UInt64": lambda p: {"data": CDRReader(p).u64()},
    "std_msgs/msg/Float32": lambda p: {"data": CDRReader(p).f32()},
    "std_msgs/msg/Float64": lambda p: {"data": CDRReader(p).f64()},
    "sensor_msgs/msg/JointState": decode_joint_state,
    "geometry_msgs/msg/Vector3": lambda p: _vector3(CDRReader(p)),
    "geometry_msgs/msg/Quaternion": lambda p: _quaternion(CDRReader(p)),
    "geometry_msgs/msg/Pose": lambda p: _pose(CDRReader(p)),
    "geometry_msgs/msg/PoseStamped": decode_pose_stamped,
    "geometry_msgs/msg/Twist": lambda p: _twist(CDRReader(p)),
    "nav_msgs/msg/Odometry": decode_odometry,
    "sensor_msgs/msg/Imu": decode_imu,
    "sensor_msgs/msg/Image": decode_image,
    "sensor_msgs/msg/CompressedImage": decode_compressed_image,
}


def decode_message(ros2_type, payload):
    """Returns a nested dict for a known ros2_type, or None if there's no
    decoder for it (the caller should fall back to a raw text view)."""

    decoder = DECODERS.get(ros2_type)
    if decoder is None:
        return None
    return decoder(payload)


def _format_scalar(v):
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def _format_list(values, indent):
    if len(values) > 16:
        shown = ", ".join(_format_scalar(v) for v in values[:16])
        return f"[{shown}, ... ({len(values)}개)]"
    return "[" + ", ".join(_format_scalar(v) for v in values) + "]"


def format_message(value, indent=0):
    """Renders a decoded message dict as indented `key: value` text, similar
    in spirit to `ros2 topic echo`'s YAML-ish output (long arrays are
    truncated instead of dumped in full)."""

    pad = "  " * indent

    if isinstance(value, dict):
        lines = []
        for key, v in value.items():
            if isinstance(v, dict):
                lines.append(f"{pad}{key}:")
                lines.append(format_message(v, indent + 1))
            elif isinstance(v, list):
                lines.append(f"{pad}{key}: {_format_list(v, indent + 1)}")
            else:
                lines.append(f"{pad}{key}: {_format_scalar(v)}")
        return "\n".join(lines)

    return f"{pad}{_format_scalar(value)}"
