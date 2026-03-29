from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # HistoryPolicy 추가
from typing import Any, Dict

# --- QoS 관련 유틸리티 ---

def get_qos_profile_from_config(sub_config: Dict[str, Any]) -> QoSProfile:
    """YAML 설정 딕셔너리를 ROS 2 QoSProfile 객체로 변환"""
    depth = sub_config.get('depth', 10)
    reliability_str = str(sub_config.get('reliability', 'reliable')).lower()
    
    profile = QoSProfile(depth=depth)
    
    # Reliability 설정
    if reliability_str == 'best_effort':
        profile.reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        profile.reliability = ReliabilityPolicy.RELIABLE
        
    # History 설정 (기본적으로 최신 N개 유지)
    profile.history = HistoryPolicy.KEEP_LAST
    
    return profile

def create_generic_sub(node: Node, msg_type: Any, sub_config: Dict[str, Any], 
                       callback_func: Any, callback_group: Any = None):
    """설정 파일을 기반으로 구독(Subscription)을 생성하여 반환"""
    topic_name = sub_config.get('topic')
    if not topic_name:
        node.get_logger().error(f"Topic name is missing in config for {msg_type.__name__}!")
        return None
        
    qos_profile = get_qos_profile_from_config(sub_config)
    
    sub = node.create_subscription(
        msg_type,
        topic_name,
        callback_func,
        qos_profile,
        callback_group=callback_group
    )
    node.get_logger().info(f"Successfully subscribed to: {topic_name} [{msg_type.__name__}]")
    return sub

# --- 파라미터 로딩 관련 유틸리티 ---

def get_required_param(node: Node, param_name: str) -> Any:
    """필수 파라미터를 가져오며, 없으면 RuntimeError 발생"""
    if not node.has_parameter(param_name):
        # 파라미터가 선언되지 않았다면 None으로 선언 시도 (YAML 로드 보장)
        node.declare_parameter(param_name, None)
    
    param = node.get_parameter(param_name)
    val = param.value
    
    if val is None:
        error_msg = f" Mandatory parameter '{param_name}' is missing in YAML config!"
        node.get_logger().fatal(error_msg)
        raise RuntimeError(error_msg)
    return val

def parse_group_parameters(node: Node, group_name: str, keys: list) -> Dict[str, Any]:
    """
    특정 그룹(예: 'vision.rgb') 하위의 여러 키값을 한 번에 딕셔너리로 파싱합니다.
    사용 예: parse_group_parameters(node, 'vision.rgb', ['topic', 'reliability', 'depth'])
    """
    group_config = {}
    for key in keys:
        full_path = f"{group_name}.{key}"
        group_config[key] = get_required_param(node, full_path)
    return group_config

# --- 기타 유틸리티 ---

def log_api_info(node: Node, message: str):
    """일관된 포맷으로 API 관련 정보를 출력"""
    node.get_logger().info(f"[KaairAPI] {message}")

def log_api_warn(node: Node, message: str):
    """일관된 포맷으로 API 관련 경고를 출력"""
    node.get_logger().warn(f"[KaairAPI] {message}")