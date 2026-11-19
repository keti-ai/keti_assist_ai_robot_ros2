"""
Finds image topics on the zenoh bridge in two complementary ways:

1. Admin-route lookup ("@/*/ros2/route/**", the same technique as
   keti_assist_ai_robot_ros2/exercise_python/zenoh_topic_list.py): finds
   topics that go through the ROS2<->zenoh DDS bridge and reports their
   declared ros2_type, even if nothing is published during the scan.

2. Active probing: briefly subscribes to every key ("**") and tries to
   CDR-decode each payload as sensor_msgs/msg/Image or
   sensor_msgs/msg/CompressedImage. This is what actually finds flat-named
   topics published directly over zenoh (bypassing the DDS bridge entirely)
   - e.g. zenoh_image_view.py's own default topic, "d455_1_rgb_image", never
   shows up in the admin routes but streams fine. It only finds topics that
   are actively publishing during the scan.

The two are merged by zenoh key expression; a topic is "supported" (can be
streamed by this app) once either method confirms it decodes as a raw or
compressed sensor_msgs image.
"""

import json
import threading
import time

import zenoh

from .image_decoder import decode_ros_image_any

# Both raw (Image) and compressed (CompressedImage) are decodable -
# image_decoder.decode_ros_image_any tries one then the other.
SUPPORTED_ROS2_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
KNOWN_ROS2_IMAGE_TYPES = SUPPORTED_ROS2_TYPES


def _open_session(host, port):
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([f"tcp/{host}:{port}"]))
    return zenoh.open(config)


def _scan_admin_routes(session, timeout):
    topics = {}

    for reply in session.get("@/*/ros2/route/topic/pub/**", timeout=timeout):
        if reply.ok is None:
            continue

        try:
            info = json.loads(bytes(reply.ok.payload.to_bytes()))
        except ValueError:
            continue

        ros2_type = info.get("ros2_type", "")
        if ros2_type not in KNOWN_ROS2_IMAGE_TYPES:
            continue

        key_expr = info.get("zenoh_key_expr")
        if not key_expr:
            continue

        topics[key_expr] = {
            "key_expr": key_expr,
            "name": info.get("ros2_name", key_expr),
            "type": ros2_type,
            "supported": ros2_type in SUPPORTED_ROS2_TYPES,
            "width": None,
            "height": None,
        }

    return topics


def _probe_live_keys(session, scan_time):
    samples = {}
    lock = threading.Lock()

    def on_sample(sample):
        key = str(sample.key_expr)
        with lock:
            if key not in samples:
                samples[key] = sample.payload.to_bytes()

    with session.declare_subscriber("**", on_sample):
        time.sleep(scan_time)

    topics = {}
    for key_expr, payload in samples.items():
        try:
            _img, info = decode_ros_image_any(payload)
        except Exception:
            continue

        topics[key_expr] = {
            "key_expr": key_expr,
            "name": key_expr,
            "type": info["ros2_type"],
            "supported": True,
            "width": info["width"],
            "height": info["height"],
        }

    return topics


def scan_image_topics(host, port, admin_timeout=3.0, probe_time=3.5):
    with _open_session(host, port) as session:
        admin_topics = _scan_admin_routes(session, admin_timeout)
        probed_topics = _probe_live_keys(session, probe_time)

    merged = dict(admin_topics)
    for key_expr, probed in probed_topics.items():
        existing = merged.get(key_expr)
        if existing is None:
            merged[key_expr] = probed
        else:
            # A successful decode always confirms support and gives real
            # dimensions; a ros2_name from the admin route is kept if present.
            existing["supported"] = True
            existing["width"] = probed["width"]
            existing["height"] = probed["height"]

    return sorted(merged.values(), key=lambda t: t["name"])
