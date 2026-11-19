"""
One zenoh subscriber per topic, shared by every browser tab watching it.
If the topic's ros2_type is one msg_decoders.py knows how to decode, each
message is shown with its actual fields (like `ros2 topic echo`); otherwise
it falls back to a best-effort raw text view, since there is no ROS2
install here to introspect arbitrary message definitions from.
"""

import json
import queue
import threading
import time

from .msg_decoders import decode_message, format_message


def _format_raw_payload(payload):
    text = payload.decode("utf-8", errors="replace")
    text = "".join(ch if ch.isprintable() or ch in "\n\t" else "." for ch in text)

    if len(text) > 400:
        text = text[:400] + "..."

    return f"[raw, 알 수 없는 타입] {text}"


def _format_payload(ros2_type, payload):
    if ros2_type:
        try:
            decoded = decode_message(ros2_type, payload)
        except Exception:
            decoded = None

        if decoded is not None:
            return format_message(decoded)

    return _format_raw_payload(payload)


class TopicEchoStream:
    def __init__(self, key_expr, ros2_type, zenoh_host, zenoh_port):
        self.key_expr = key_expr
        self.ros2_type = ros2_type
        self.zenoh_host = zenoh_host
        self.zenoh_port = zenoh_port

        self._condition = threading.Condition()
        self._message = None
        self._version = 0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_latest(self, last_version, timeout=1.0):
        with self._condition:
            if self._version == last_version:
                self._condition.wait(timeout=timeout)
            return self._message, self._version

    def _publish(self, message):
        with self._condition:
            self._message = message
            self._version += 1
            self._condition.notify_all()

    def _run(self):
        import zenoh  # imported lazily so a missing driver only breaks echoing

        samples = queue.Queue(maxsize=1)

        def on_sample(sample):
            payload = sample.payload.to_bytes()
            try:
                samples.put_nowait(payload)
            except queue.Full:
                try:
                    samples.get_nowait()
                except queue.Empty:
                    pass
                try:
                    samples.put_nowait(payload)
                except queue.Full:
                    pass

        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        endpoint = f"tcp/{self.zenoh_host}:{self.zenoh_port}"
        config.insert_json5("connect/endpoints", json.dumps([endpoint]))

        print(f"[topic_echo] {self.key_expr}: connecting to {endpoint}")

        while True:
            try:
                with zenoh.open(config) as session:
                    with session.declare_subscriber(self.key_expr, on_sample):
                        print(f"[topic_echo] {self.key_expr}: subscribed")

                        while True:
                            try:
                                payload = samples.get(timeout=0.5)
                            except queue.Empty:
                                continue

                            self._publish({
                                "timestamp": time.time(),
                                "size": len(payload),
                                "text": _format_payload(self.ros2_type, payload),
                            })
            except Exception as e:
                print(f"[topic_echo] {self.key_expr}: connection error: {e}. Retrying in 2s...")
                time.sleep(2)


class TopicEchoManager:
    def __init__(self, zenoh_host, zenoh_port):
        self.zenoh_host = zenoh_host
        self.zenoh_port = zenoh_port
        self._streams = {}
        self._lock = threading.Lock()

    def get_stream(self, key_expr, ros2_type=None):
        with self._lock:
            stream = self._streams.get(key_expr)
            if stream is None:
                stream = TopicEchoStream(key_expr, ros2_type, self.zenoh_host, self.zenoh_port)
                self._streams[key_expr] = stream
            return stream
