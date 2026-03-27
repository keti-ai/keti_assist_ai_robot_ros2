from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image

class VisionProxy:
    def __init__(self, node : Node):
        self.node = node 
        self.vision_cb_group = MutuallyExclusiveCallbackGroup()
        self.latest_msg: Image | None = None
        # 데이터 유효 시간 설정 (예: 0.5초 이상 지난 데이터는 버림)
        self.max_latency_sec = 0.1

        # QoS 설정 (Best Effort, Depth 1)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.node.create_subscription(
            Image, 
            '/femto/color/image_raw', 
            self._image_callback, 
            qos,
            callback_group=self.vision_cb_group
            )
        self.node.get_logger().info(f"✅ VisionProxy: 최신성 검사 활성화 (Max Latency: {self.max_latency_sec}s)")

    def _image_callback(self, msg):
        self.latest_msg = msg

    def get_latest_image_msg(self, is_strict=True):
        """
        최신 이미지를 반환.
        is_strict=True(기본값): 지연 시간이 max_latency_sec을 초과하면 None 반환 및 경고.
        is_strict=False: 지연이 있더라도 현재 가지고 있는 가장 최신 메시지 반환.
        """
        if self.latest_msg is None:
            return None

        # 1. 현재 노드 시간과 메시지 타임스탬프 비교
        now = self.node.get_clock().now()
        msg_time = Time.from_msg(self.latest_msg.header.stamp)
        
        # 2. 지연 시간(Latency) 계산
        diff = now - msg_time
        latency_sec = diff.nanoseconds / 1e9

        # 3. 최신성 체크
        if latency_sec > self.max_latency_sec:
            self.node.get_logger().warn(
                f"⚠️ 이미지 데이터 지연 발생! (지연 시간: {latency_sec:.3f}s)"
            )
            
            # 엄격 모드인 경우 데이터가 있어도 None 반환 (안전 우선)
            if is_strict:
                self.node.get_logger().error("🚫 [Strict Mode] 오래된 데이터를 폐기합니다.")
                return None
            
        return self.latest_msg