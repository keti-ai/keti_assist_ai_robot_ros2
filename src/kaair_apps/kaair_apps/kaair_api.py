import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from typing import TYPE_CHECKING, Optional
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
try:
    # ros2 node를 통한 실행시 임포트 경로
    from .modules.vision_proxy import VisionProxy
    from .modules.tf_proxy import TFProxy
    from .modules.controller_proxy import ControllerProxy
    from .utils import load_yaml_config, resolve_config_path
except (ImportError, ValueError):
    # python 직접 실행시 임포트 경로
    from kaair_apps.modules.vision_proxy import VisionProxy
    from kaair_apps.modules.tf_proxy import TFProxy
    from kaair_apps.modules.controller_proxy import ControllerProxy
    from kaair_apps.utils import load_yaml_config, resolve_config_path
    
    
class KaairRobotAPI(Node):
    def __init__(self, config_input: str, filename: str = 'app_params.yaml'):
        super().__init__('kaair_api_node')

        # 프록시 객체의 타입 정의 
        self.vision: Optional['VisionProxy'] = None
        self.tf: Optional['TFProxy'] = None
        self.controller: Optional['ControllerProxy'] = None

        # Callback 그룹 사전 정의. 각 Proxy 코드에서 사용 가능
        self.cb_control_group = ReentrantCallbackGroup()    # joint_states등 제어부 콜백그룹
        self.cb_image_group = MutuallyExclusiveCallbackGroup()  # rgb,depth등 이미지 콜백그룹
        self.cb_pc_group = MutuallyExclusiveCallbackGroup() # Point Cloud를 위한 콜백그룹

        try:
            # 유틸리티를 통해 실제 경로 확보
            final_path = resolve_config_path(config_input, filename)
            self.get_logger().info(f"Using config file: {final_path}")
            
            # YAML 로드 및 주입
            config_dict = load_yaml_config(final_path)

            # 각 프록시 클래스 선언 및 yaml 파일 인자 전달
            self.vision = VisionProxy(self, config_dict.get('vision_node', {}))
            self.tf = TFProxy(self, config_dict.get('tf_node', {}))
            self.controller = ControllerProxy(self, config_dict.get('controller_node', {}))

            self.get_logger().info("Kaair Robot API initialized successfully.")
            
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize: {e}")
            raise e


import cv2
import os
import time
from cv_bridge import CvBridge
from rclpy.time import Time

# 이미지 변환용 브릿지
bridge = CvBridge()

def terminal_menu(node : Node):
    """사용자로부터 명령을 입력받아 실행하는 루프"""
    print("\n" + "="*40)
    print(" 🚀 Kaair API Snapshot & Sync Test")
    print(" 1: Capture RGB (Now) - 즉시 반환")
    print(" 2: Capture RGB (Latest) - 호출 이후 대기")
    print(" q: Quit")
    print("="*40)

    save_dir = "captures"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    while rclpy.ok():
        user_input = input("\n[Enter Command]: ").strip().lower()

        # --- 1. 요청 시점 기록 (ROS Time 기준) ---
        request_time = node.get_clock().now()
        req_sec = request_time.to_msg().sec
        req_nanosec = request_time.to_msg().nanosec

        img_msg = None
        mode_str = ""

        if user_input == '1':
            mode_str = "NOW (Non-blocking)"
            print(f"📡 [{mode_str}] 요청 시점: {req_sec}.{req_nanosec:09d}")
            img_msg = node.vision.get_rgb(is_latest=False)
            
        elif user_input == '2':
            mode_str = "LATEST (Blocking Wait)"
            timeout = 1.0
            print(f"⏳ [{mode_str}] 요청 시점: {req_sec}.{req_nanosec:09d}")
            print(f"   ㄴ 새 데이터 대기 중... (Max {timeout}s)")
            img_msg = node.vision.get_rgb(is_latest=True, timeout_sec=timeout)

        elif user_input == '3':
            target_angles = [1.57, -0.6] 
            print(f"👀 헤드 이동 요청 -> {target_angles}")
            node.controller.set_head(target_angles, duration_sec=3)

        elif user_input == '4':
            target_angles = [-1.57, 0.3] 
            print(f"👀 헤드 이동 요청 -> {target_angles}")
            node.controller.set_head(target_angles, duration_sec=3)

        elif user_input == '5':
            print("📡 실시간 토픽 연결 상태 확인 중...")
            node.vision.check_topics_existence()

        elif user_input == 'q':
            print("테스트를 종료합니다.")
            break

        # --- 2. 결과 출력 및 저장 ---
        if img_msg:
            # 이미지 타임스탬프 추출
            msg_sec = img_msg.header.stamp.sec
            msg_nanosec = img_msg.header.stamp.nanosec
            
            # 시차 계산 (이미지 시간 - 요청 시간)
            diff = (msg_sec + msg_nanosec*1e-9) - (req_sec + req_nanosec*1e-9)
            
            print(f"✅ 데이터 취득 성공!")
            print(f"   - 이미지 타임스탬프: {msg_sec}.{msg_nanosec:09d}")
            print(f"   - 요청 대비 시차: {diff:+.4f} 초 (양수면 최신 데이터)")

            try:
                cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                file_name = f"{save_dir}/rgb_{msg_sec}_{msg_nanosec}.png"
                cv2.imwrite(file_name, cv_image)
                print(f"📂 파일 저장 완료: {file_name}")
            except Exception as e:
                print(f"❌ 이미지 처리 에러: {e}")
                
        elif user_input in ['1', '2']:
            print(f"⚠️ [{mode_str}] 결과: 데이터를 가져오지 못했습니다.")

    print("Terminal Menu Closed.")

# --- 수정된 메인 함수 ---
def main(args=None):
    rclpy.init(args=args)
    
    # 1. 노드 생성 (질문하신 경로 방식 유지)
    try:
        # 패키지 방식 혹은 절대경로 방식 선택 가능
        node = KaairRobotAPI(config_input='/ros_ws/src/kaair_apps/config/app_params.yaml')
    except Exception as e:
        print(f"❌ 초기화 실패: {e}")
        return

    # 2. MultiThreadedExecutor 설정
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 3. ROS 2 Spin을 별도 스레드에서 실행 (백그라운드)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # 4. 메인 스레드에서 터미널 메뉴 실행
        terminal_menu(node)

    except Exception as e:
        print(f"❌ 실행 중 에러 발생: {e}")
    finally:
        # 종료 처리
        node.destroy_node()
        rclpy.try_shutdown()
        # 스레드가 안전하게 멈출 수 있도록 대기 (선택 사항)
        # spin_thread.join()

if __name__ == '__main__':
    main()