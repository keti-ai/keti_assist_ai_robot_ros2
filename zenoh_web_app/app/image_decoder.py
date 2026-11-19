"""
ROS2 CDR deserializer for sensor_msgs/msg/Image, adapted from
keti_assist_ai_robot_ros2/exercise_python/zenoh_image_view.py.
"""

import struct

import cv2
import numpy as np


class CDRReader:
    def __init__(self, data):
        self.data = memoryview(data)

        if len(data) < 4:
            raise ValueError("CDR payload too short")

        identifier = bytes(self.data[:2])

        if identifier == b"\x00\x01":
            self.endian = "<"
        elif identifier == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError(f"Unsupported CDR encapsulation: {identifier.hex()}")

        self.pos = 4
        self.base = 4

    def align(self, size):
        relative = self.pos - self.base
        self.pos += (-relative) % size

    def read(self, fmt, alignment):
        self.align(alignment)
        size = struct.calcsize(fmt)

        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR payload")

        value = struct.unpack_from(self.endian + fmt, self.data, self.pos)[0]
        self.pos += size
        return value

    def u8(self):
        return self.read("B", 1)

    def u32(self):
        return self.read("I", 4)

    def i32(self):
        return self.read("i", 4)

    def string(self):
        length = self.u32()

        if length < 1 or self.pos + length > len(self.data):
            raise ValueError("Invalid CDR string")

        result = bytes(self.data[self.pos:self.pos + length])
        self.pos += length
        return result.rstrip(b"\x00").decode("utf-8")

    def byte_array(self):
        length = self.u32()

        if self.pos + length > len(self.data):
            raise ValueError("Invalid CDR byte array")

        result = self.data[self.pos:self.pos + length]
        self.pos += length
        return result


CHANNELS_BY_ENCODING = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
}

# Depth encodings: single-channel, wider-than-8-bit. Raw values are meters
# (32FC1) or millimeters (16UC1); DEPTH_METERS_SCALE converts to meters.
DEPTH_DTYPE_BY_ENCODING = {
    "32FC1": np.float32,
    "16UC1": np.uint16,
}
DEPTH_METERS_SCALE = {
    "32FC1": 1.0,
    "16UC1": 1.0 / 1000.0,
}

# Clip range for depth -> grayscale normalization (meters). Close = bright.
DEPTH_NEAR_M = 0.2
DEPTH_FAR_M = 6.0


def _depth_to_bgr(raw, encoding):
    meters = raw.astype(np.float32) * DEPTH_METERS_SCALE[encoding]
    valid = np.isfinite(meters) & (meters > 0)

    normalized = np.zeros(meters.shape, dtype=np.uint8)
    clipped = np.clip(meters, DEPTH_NEAR_M, DEPTH_FAR_M)
    scaled = (clipped - DEPTH_NEAR_M) / (DEPTH_FAR_M - DEPTH_NEAR_M)
    normalized[valid] = (255 - scaled[valid] * 255).astype(np.uint8)

    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    colored[~valid] = (0, 0, 0)
    return colored


def decode_ros_image(payload):
    r = CDRReader(payload)

    # std_msgs/Header
    sec = r.i32()
    nanosec = r.u32()
    frame_id = r.string()

    # sensor_msgs/Image
    height = r.u32()
    width = r.u32()
    encoding = r.string()
    is_bigendian = r.u8()
    step = r.u32()
    data = r.byte_array()

    if height == 0 or width == 0:
        raise ValueError("Invalid image dimensions")

    if encoding in DEPTH_DTYPE_BY_ENCODING:
        dtype = DEPTH_DTYPE_BY_ENCODING[encoding]
        itemsize = np.dtype(dtype).itemsize

        if step < width * itemsize or len(data) < height * step:
            raise ValueError("Invalid image buffer or step")

        raw = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=data,
            strides=(step, itemsize),
        )
        img = _depth_to_bgr(raw, encoding)

    elif encoding in CHANNELS_BY_ENCODING:
        ch = CHANNELS_BY_ENCODING[encoding]

        if step < width * ch or len(data) < height * step:
            raise ValueError("Invalid image buffer or step")

        img = np.ndarray(
            shape=(height, width, ch),
            dtype=np.uint8,
            buffer=data,
            strides=(step, ch, 1),
        )

        if encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif encoding == "mono8":
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img = img.copy()

    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    return img, {
        "width": width,
        "height": height,
        "encoding": encoding,
        "frame_id": frame_id,
        "timestamp": sec + nanosec * 1e-9,
    }
