import rclpy
from rclpy.node import Node
import time
from rclpy.executors import MultiThreadedExecutor
try:
    from .modules.vision_proxy import VisionProxy
except:
    from modules.vision_proxy import VisionProxy

class KaairRobotAPI:
    def __init__(self, node):
        self.node = node
        self.vision = VisionProxy(self.node)
        self.node.get_logger().info("Kaair 통합 API가 성공적으로 초기화되었습니다.")

    def get_valid_raw_image(self):
        return self.vision.get_latest_image_msg()

# --- 테스트용 메인 함수 ---
def main(args=None):
    rclpy.init(args=args)
    test_node = Node('api_test_node')
    
    api = KaairRobotAPI(test_node)
    
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    
    try:
        api = KaairRobotAPI(test_node)
        test_node.get_logger().info("🚀 테스트 시작: 데이터의 최신성(Freshness)을 비교합니다.")
        
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < 5.0):
            # 1. 노드 이벤트 처리 (이미지 콜백 실행)
            rclpy.spin_once(test_node, timeout_sec=0.01)
            
            # 2. 함수 호출 시점 기록 (System Time)
            call_time = test_node.get_clock().now()
            
            # 3. 데이터 가져오기
            img_msg = api.vision.get_latest_image_msg()
            
            if img_msg:
                # 4. 이미지 타임스탬프 추출 (Message Time)
                from rclpy.time import Time
                msg_time = Time.from_msg(img_msg.header.stamp)
                
                # 5. 시간차 계산 (Latency)
                latency = call_time - msg_time
                latency_ms = latency.nanoseconds / 1e6  # 밀리초 단위 변환
                
                # 6. 상세 로깅
                test_node.get_logger().info("-" * 50)
                test_node.get_logger().info(f"✅ 데이터 수신 성공!")
                test_node.get_logger().info(f"📍 호출 시점: {call_time.nanoseconds / 1e9:.3f}s")
                test_node.get_logger().info(f"📸 촬영 시점: {msg_time.nanoseconds / 1e9:.3f}s")
                test_node.get_logger().info(f"⏱️ 지연 시간(Latency): {latency_ms:.2f} ms")
                test_node.get_logger().info("-" * 50)
                break
        else:
            test_node.get_logger().error("❌ 5초 동안 이미지 데이터를 수신하지 못했습니다.")

    except Exception as e:
        test_node.get_logger().error(f"❌ 테스트 중 에러 발생: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()