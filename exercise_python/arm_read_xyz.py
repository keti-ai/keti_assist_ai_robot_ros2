import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self, target_frame, source_frame):
        try:
            # target: 기준(예: link_base), source: 읽고 싶은 곳(예: tool_tcp)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, now)
            
            p = trans.transform.translation
            o = trans.transform.rotation
            
            # Quaternion -> RPY 변환 (scipy 사용)
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([o.x, o.y, o.z, o.w])
            rpy = r.as_euler('xyz', degrees=False)

            self.get_logger().info(f"\n📍 [{source_frame}] relative to [{target_frame}]")
            self.get_logger().info(f"XYZ: {p.x:.4f}, {p.y:.4f}, {p.z:.4f}")
            self.get_logger().info(f"RPY: {rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}")
            
            return p, rpy
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF를 읽을 수 없습니다: {e}')
            return None, None
        
def main():
    rclpy.init()
    node = PoseReader()

    node.get_logger().info("TF 데이터를 기다리는 중...")
    
    # 🌟 수정 포인트: Duration 비교 방식 변경
    time_to_wait_sec = 1.0
    start_time = node.get_clock().now()
    
    # rclpy.time.Time 간의 뺄셈 결과는 Duration 객체입니다.
    # .nanoseconds를 1,000,000,000으로 나누어 초 단위로 비교합니다.
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < time_to_wait_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 호출 예시: 프레임 이름 확인 필수 (link_base vs arm_base)
    #target = 'link_base' 
    target = 'link_base' 
    source = 'tool_tcp_link'

    p, rpy = node.get_current_pose(target, source)
    
    if p is not None:
        print("-" * 30)
        print(f"XYZ: {p.x:.4f}, {p.y:.4f}, {p.z:.4f}")
        print(f"RPY (rad): {rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}")
        print("-" * 30)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import time # 대기 시간을 위해 추가
    main()