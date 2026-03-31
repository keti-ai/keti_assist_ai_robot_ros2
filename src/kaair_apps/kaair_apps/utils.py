import yaml
import os
import threading
from typing import Any, Dict
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

class RobotActionHandle:
    def __init__(self, action_name: str):
        self.action_name = action_name
        self._event = threading.Event()
        self._result = None
        self._status = GoalStatus.STATUS_UNKNOWN
        self.accepted = False

    def wait(self, timeout: float = None) -> bool:
        """이벤트가 set될 때까지 메인 스레드를 블로킹합니다."""
        return self._event.wait(timeout=timeout)

    def set_done(self, status, result):
        """액션 결과가 나왔을 때 콜백에서 호출합니다."""
        self._status = status
        self._result = result
        self._event.set()

    @property
    def result(self):
        return self._result

    @property
    def status(self):
        return self._status

def resolve_config_path(path_or_pkg: str, filename: str = None) -> str:
    """
    1. path_or_pkg가 실제 존재하는 경로라면 그 경로를 반환 (절대/상대 경로)
    2. 경로가 없다면 path_or_pkg를 패키지명으로 간주하여 share 폴더에서 filename을 찾음
    """
    # 방식 A: 직접 경로 (절대경로 혹은 현재 작업 디렉토리 기준 상대경로)
    if os.path.exists(path_or_pkg):
        if os.path.isdir(path_or_pkg) and filename:
            full_path = os.path.join(path_or_pkg, filename)
            if os.path.exists(full_path):
                return full_path
        elif os.path.isfile(path_or_pkg):
            return path_or_pkg

    # 방식 B: ROS 2 패키지 기반 찾기
    try:
        package_share_directory = get_package_share_directory(path_or_pkg)
        # share/<pkg>/config/<filename> 경로 탐색
        config_path = os.path.join(package_share_directory, 'config', filename)
        if os.path.exists(config_path):
            return config_path
    except Exception:
        pass

    raise FileNotFoundError(f"Could not resolve config path: {path_or_pkg} / {filename}")

def load_yaml_config(full_path: str) -> Dict[str, Any]:
    """YAML 파일을 읽어 딕셔너리로 반환 (ROS 2 파라미터 구조 대응)"""
    with open(full_path, 'r') as f:
        data = yaml.safe_load(f)
        
    # ROS 2 특유의 node_name: ros__parameters 계층 제거 로직
    if isinstance(data, dict) and len(data) == 1:
        first_key = list(data.keys())[0]
        if isinstance(data[first_key], dict) and 'ros__parameters' in data[first_key]:
            return data[first_key]['ros__parameters']
            
    return data

def create_generic_sub(node: Node, msg_type: Any, sub_config: Dict[str, Any], 
                       callback_func: Any, callback_group: Any = None):
    """주입받은 딕셔너리 기반 구독 생성 (Durability 설정 추가)"""
    topic_name = sub_config.get('topic')
    depth = sub_config.get('depth', 10)
    
    # 1. 설정값 읽기 및 소문자 정규화
    reliability = str(sub_config.get('reliability', 'reliable')).lower()
    durability = str(sub_config.get('durability', 'volatile')).lower() # 기본값 volatile
    
    # 2. QoS 프로파일 생성
    qos = QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST)
    
    # Reliability 설정 (Best Effort vs Reliable)
    qos.reliability = (ReliabilityPolicy.BEST_EFFORT if reliability == 'best_effort' 
                       else ReliabilityPolicy.RELIABLE)
    
    # Durability 설정 (Transient Local vs Volatile)
    # YAML에 'transient_local'이 명시된 경우에만 설정, 나머지는 모두 Volatile
    if durability == 'transient_local':
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        qos.durability = DurabilityPolicy.VOLATILE
    
    return node.create_subscription(
        msg_type, 
        topic_name, 
        callback_func, 
        qos, 
        callback_group=callback_group
    )