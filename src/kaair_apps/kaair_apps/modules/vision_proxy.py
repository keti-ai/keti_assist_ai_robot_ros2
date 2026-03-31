from typing import Optional, Any, TYPE_CHECKING
from rclpy.time import Time
import time
from sensor_msgs.msg import Image, PointCloud2

from kaair_apps.utils import create_generic_sub

# VSCode 지능형 안내를 위해 Main 클래스 타입을 가져옴
if TYPE_CHECKING:
    from ..kaair_api import KaairRobotAPI


class VisionProxy:
    def __init__(self, api: 'KaairRobotAPI', config: dict):
        self.node = api 

        self.latest_color: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_points: Optional[PointCloud2] = None

        # 설정 추출 및 구독 (QoS는 utils 내부에서 처리)
        self.sub_rgb = create_generic_sub(self.node, Image, config.get('color', {}), 
                                          self._rgb_cb, callback_group=self.node.cb_image_group)
        self.sub_depth = create_generic_sub(self.node, Image, config.get('depth', {}), 
                                            self._depth_cb, callback_group=self.node.cb_image_group)
        self.sub_points = create_generic_sub(self.node, PointCloud2, config.get('points', {}), 
                                             self._points_cb, callback_group=self.node.cb_pc_group)
        
        self.node.get_logger().info("VisionProxy: 최신성 검사 대기 로직 활성화")
        # 초기화 시점에 토픽 존재 여부를 점검 (로그 출력용)
        self.check_topics_existence()


    def check_topics_existence(self) -> bool:
        """
        설정된 토픽에 대해 실제로 데이터를 보내주는 Publisher가 있는지 확인합니다.
        """
        all_exists = True
        print("\n🔍 --- Vision System Connectivity Check ---")
        
        target_sensors = {
            'RGB': self.sub_rgb,
            'Depth': self.sub_depth,
            'Points': self.sub_points
        }

        for name, sub in target_sensors.items():
            topic_name = sub.topic_name
            # 현재 이 토픽을 퍼블리싱하고 있는 노드의 개수를 파악
            count = self.node.count_publishers(topic_name)
            
            if count > 0:
                print(f"✅ {name:6}: [{topic_name}] - Active (Publishers: {count})")
            else:
                print(f"❌ {name:6}: [{topic_name}] - OFFLINE (No Publishers found)")
                all_exists = False
        
        print("-------------------------------------------\n")
        
        if not all_exists:
            self.node.get_logger().error("카메라 노드가 죽어있거나 토픽명이 일치하지 않습니다!")
            
        return all_exists

    # --- Internal Callbacks ---
    def _rgb_cb(self, msg: Image): self.latest_color = msg
    def _depth_cb(self, msg: Image): self.latest_depth = msg
    def _points_cb(self, msg: PointCloud2): self.latest_points = msg

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
        clock = self.node.get_clock()
        request_time = clock.now()
        
        start_wait_time = time.time() # 시스템 시간 기준
        
        while (time.time() - start_wait_time) < timeout_sec:
            current_msg = getattr(self, attr_name)
            
            if current_msg is not None:
                # 메시지의 타임스탬프가 요청 시점보다 '이후'인지 확인
                msg_time = Time.from_msg(current_msg.header.stamp)
                if msg_time > request_time:
                    return current_msg
            
            # 10ms 대기 (100Hz 상당) - 새로운 스레드를 생성하지 않음
            time.sleep(0.01)
            
        self.node.get_logger().warn(f"VisionProxy: [{attr_name}] 새 데이터 수신 타임아웃 ({timeout_sec}s)")
        return None