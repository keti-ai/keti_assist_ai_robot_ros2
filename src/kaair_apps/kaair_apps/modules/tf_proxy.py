from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from typing import TYPE_CHECKING, Optional

# VSCode 인텔리센스를 위한 타입 힌트 (실행 시에는 무시됨)
if TYPE_CHECKING:
    from ..kaair_api import KaairRobotAPI

class TFProxy:
    def __init__(self, api: 'KaairRobotAPI'):
        """
        TFProxy 초기화
        :param api: Main API 객체 (KaairRobotAPI 인스턴스)
        """
        self.api = api
        self.node = api  # KaairRobotAPI가 Node를 상속받았으므로 바로 사용
        
        # TF 전용 버퍼 및 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.node.get_logger().info("TFProxy: 초기화 완료 (base_footprint -> arm_base 추적 시작)")

    def get_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        """기본 좌표 변환 획득 함수"""
        try:
            # 최신(Time(0)) 변환 정보를 0.1초 동안 기다려 가져옴
            return self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                Time(), 
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None

    def get_arm_base_pose(self) -> Optional[TransformStamped]:
        """
        로봇의 기준점(base_footprint)으로부터 로봇 팔의 기지점(arm_base) 위치를 계산
        :return: TransformStamped 메시지 또는 None
        """
        # base_footprint 기준 arm_base의 위치 계산
        transform = self.get_transform('base_footprint', 'arm_base')
        
        if transform:
            # 인텔리센스를 위해 로그 출력 예시
            # t = transform.transform.translation
            # self.node.get_logger().info(f"Arm Base Pos: x={t.x:.2f}, y={t.y:.2f}, z={t.z:.2f}")
            pass
            
        return transform