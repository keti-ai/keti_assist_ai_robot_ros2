
import argparse
import json
import queue
import struct
import time

import cv2
import numpy as np
import zenoh


# ----------------------------------------
# ROS2 CDR Deserializer
# sensor_msgs/msg/Image
# ----------------------------------------

class CDRReader:
    def __init__(self, data):
        self.data = memoryview(data)

        if len(data) < 4:
            raise ValueError("CDR payload too short")

        # CDR encapsulation identifier
        # 00 01: little-endian
        # 00 00: big-endian
        identifier = bytes(self.data[:2])

        if identifier == b"\x00\x01":
            self.endian = "<"
        elif identifier == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError(
                f"Unsupported CDR encapsulation: {identifier.hex()}"
            )

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

        value = struct.unpack_from(
            self.endian + fmt,
            self.data,
            self.pos
        )[0]

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

        result = bytes(
            self.data[self.pos:self.pos + length]
        )

        self.pos += length
        return result.rstrip(b"\x00").decode("utf-8")

    def byte_array(self):
        length = self.u32()

        if self.pos + length > len(self.data):
            raise ValueError("Invalid CDR byte array")

        result = self.data[self.pos:self.pos + length]
        self.pos += length
        return result


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

    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }

    if encoding not in channels:
        raise ValueError(f"Unsupported encoding: {encoding}")

    ch = channels[encoding]

    if height == 0 or width == 0:
        raise ValueError("Invalid image dimensions")

    if step < width * ch or len(data) < height * step:
        raise ValueError("Invalid image buffer or step")

    # Respect row padding indicated by step
    img = np.ndarray(
        shape=(height, width, ch),
        dtype=np.uint8,
        buffer=data,
        strides=(step, ch, 1),
    )

    # Convert ROS encoding to OpenCV BGR
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

    return img, {
        "width": width,
        "height": height,
        "encoding": encoding,
        "frame_id": frame_id,
        "timestamp": sec + nanosec * 1e-9,
    }


# ----------------------------------------
# Zenoh Image Subscriber
# ----------------------------------------

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Zenoh Bridge IP or hostname"
    )

    parser.add_argument(
        "--port",
        type=int,
        default=7447
    )

    parser.add_argument(
        "--topic",
        default="camera/color/image_raw",
        help="Zenoh key expression"
    )

    args = parser.parse_args()

    frames = queue.Queue(maxsize=1)
    received = 0

    def on_image(sample):
        nonlocal received

        # Keep Zenoh callback short
        payload = sample.payload.to_bytes()
        received += 1

        try:
            frames.put_nowait(payload)
        except queue.Full:
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
            try:
                frames.put_nowait(payload)
            except queue.Full:
                pass

    config = zenoh.Config()

    config.insert_json5(
        "mode",
        '"client"'
    )

    endpoint = f"tcp/{args.host}:{args.port}"

    config.insert_json5(
        "connect/endpoints",
        json.dumps([endpoint])
    )

    print(f"Connecting to {endpoint}")
    print(f"Topic: {args.topic}")

    with zenoh.open(config) as session:
        with session.declare_subscriber(
            args.topic,
            on_image
        ):
            print("Subscribed. Press Q or ESC to exit.")

            last_time = time.perf_counter()
            shown_frames = 0
            fps = 0.0

            try:
                while True:
                    try:
                        payload = frames.get(timeout=0.03)
                    except queue.Empty:
                        # Keep OpenCV UI responsive
                        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                            break
                        continue

                    try:
                        img, info = decode_ros_image(payload)
                    except Exception as e:
                        print(f"Decode error: {e}")
                        continue

                    shown_frames += 1

                    now = time.perf_counter()
                    elapsed = now - last_time

                    if elapsed >= 1.0:
                        fps = shown_frames / elapsed
                        shown_frames = 0
                        last_time = now

                        print(
                            f"FPS: {fps:.1f} | "
                            f"{info['width']}x{info['height']} | "
                            f"{info['encoding']} | "
                            f"Received: {received}"
                        )

                    cv2.putText(
                        img,
                        f"Zenoh {fps:.1f} FPS",
                        (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2
                    )

                    cv2.imshow("Zenoh RGB Camera", img)

                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break

            except KeyboardInterrupt:
                pass

            finally:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
