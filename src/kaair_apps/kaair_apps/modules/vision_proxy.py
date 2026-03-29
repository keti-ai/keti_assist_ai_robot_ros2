from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, PointCloud2
from typing import Optional, Any, Union

from kaair_apps.utils import create_generic_sub


class VisionProxy:
    def __init__(self, node: Node, config: dict):
        self.node = node 
        # 콜백 그룹: 센서 데이터 수신을 독립적으로 처리
        self.cb_groups = [MutuallyExclusiveCallbackGroup() for _ in range(3)]
        
        self.latest_color: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_points: Optional[PointCloud2] = None

        # 가독성을 위해 토픽 설정 추출
        rgb_cfg = config.get('rgb', {})
        depth_cfg = config.get('depth', {})
        points_cfg = config.get('points', {})

        self.sub_rgb = create_generic_sub(
            self.node, Image, rgb_cfg, 
            self._rgb_cb, callback_group=self.cb_groups[0]
        )
        self.sub_depth = create_generic_sub(
            self.node, Image, depth_cfg, 
            self._depth_cb, callback_group=self.cb_groups[1]
        )
        self.sub_points = create_generic_sub(
            self.node, PointCloud2, points_cfg, 
            self._points_cb, callback_group=self.cb_groups[2]
        )

        self.node.get_logger().info("VisionProxy: 최신성 검사 대기 로직 활성화")

    # --- Internal Callbacks ---
    def _rgb_cb(self, msg: Image):
        self.latest_color = msg

    def _depth_cb(self, msg: Image):
        self.latest_depth = msg

    def _points_cb(self, msg: PointCloud2):
        self.latest_points = msg

    # --- Public Data Access API ---
    
    # 1. 기존의 즉시 반환(Non-blocking) 메서드들
    def get_rgb(self, is_latest: bool = False, timeout_sec: float = 1.0) -> Optional[Image]:
        """
        is_latest=True: 호출 시점 이후의 새로운 데이터를 기다림 (Blocking)
        is_latest=False: 현재 가지고 있는 가장 최근 데이터를 즉시 반환 (Non-blocking)
        """
        if not is_latest:
            return self.latest_color
        return self._wait_for_msg('latest_color', timeout_sec)

    def get_depth(self, is_latest: bool = False, timeout_sec: float = 1.0) -> Optional[Image]:
        if not is_latest:
            return self.latest_depth
        return self._wait_for_msg('latest_depth', timeout_sec)

    def get_points(self, is_latest: bool = False, timeout_sec: float = 1.0) -> Optional[PointCloud2]:
        if not is_latest:
            return self.latest_points
        return self._wait_for_msg('latest_points', timeout_sec)

    # --- Core Logic ---
    def _wait_for_msg(self, attr_name: str, timeout_sec: float) -> Optional[Any]:
        """
        공통 대기 로직: 호출 시점보다 큰 타임스탬프를 가진 데이터가 올 때까지 Polling
        """
        clock = self.node.get_clock()
        request_time = clock.now()
        timeout_duration = Duration(seconds=timeout_sec)
        
        # 100Hz 체크 주기
        check_rate = self.node.create_rate(100) 
        start_wait_time = clock.now()
        
        while (clock.now() - start_wait_time) < timeout_duration:
            current_msg = getattr(self, attr_name)
            
            if current_msg is not None:
                msg_time = Time.from_msg(current_msg.header.stamp)
                # 메시지 시간이 요청 시점보다 '이후'인 경우에만 성공으로 간주
                if msg_time > request_time:
                    return current_msg
            
            check_rate.sleep()
            
        self.node.get_logger().warn(f"VisionProxy: [{attr_name}] 새 데이터 수신 타임아웃 ({timeout_sec}s)")
        return None