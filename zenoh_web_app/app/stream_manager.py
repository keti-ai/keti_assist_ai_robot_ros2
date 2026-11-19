"""
One zenoh subscriber per topic, shared by every browser tile watching it.
Each TopicStream keeps only the latest decoded frame (as JPEG) and lets any
number of MJPEG generators wait on it.
"""

import json
import queue
import threading
import time

import cv2

from .image_decoder import decode_ros_image


class TopicStream:
    def __init__(self, key_expr, zenoh_host, zenoh_port):
        self.key_expr = key_expr
        self.zenoh_host = zenoh_host
        self.zenoh_port = zenoh_port

        self._condition = threading.Condition()
        self._jpeg = None
        self._version = 0
        self._last_error = None

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_latest(self, last_version, timeout=1.0):
        with self._condition:
            if self._version == last_version:
                self._condition.wait(timeout=timeout)
            return self._jpeg, self._version

    def _publish(self, jpeg_bytes):
        with self._condition:
            self._jpeg = jpeg_bytes
            self._version += 1
            self._condition.notify_all()

    def _run(self):
        import zenoh  # imported lazily so a missing driver only breaks streaming

        frames = queue.Queue(maxsize=1)

        def on_sample(sample):
            payload = sample.payload.to_bytes()
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
        config.insert_json5("mode", '"client"')
        endpoint = f"tcp/{self.zenoh_host}:{self.zenoh_port}"
        config.insert_json5("connect/endpoints", json.dumps([endpoint]))

        print(f"[stream] {self.key_expr}: connecting to {endpoint}")

        while True:
            try:
                with zenoh.open(config) as session:
                    with session.declare_subscriber(self.key_expr, on_sample):
                        print(f"[stream] {self.key_expr}: subscribed")

                        while True:
                            try:
                                payload = frames.get(timeout=0.5)
                            except queue.Empty:
                                continue

                            try:
                                img, _info = decode_ros_image(payload)
                            except Exception as e:
                                self._last_error = str(e)
                                continue

                            ok, jpeg = cv2.imencode(
                                ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80]
                            )
                            if ok:
                                self._publish(jpeg.tobytes())
            except Exception as e:
                self._last_error = str(e)
                print(f"[stream] {self.key_expr}: connection error: {e}. Retrying in 2s...")
                time.sleep(2)


class StreamManager:
    def __init__(self, zenoh_host, zenoh_port):
        self.zenoh_host = zenoh_host
        self.zenoh_port = zenoh_port
        self._streams = {}
        self._lock = threading.Lock()

    def get_stream(self, key_expr):
        with self._lock:
            stream = self._streams.get(key_expr)
            if stream is None:
                stream = TopicStream(key_expr, self.zenoh_host, self.zenoh_port)
                self._streams[key_expr] = stream
            return stream
