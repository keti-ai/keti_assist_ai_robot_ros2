import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from scipy.spatial.transform import Rotation as R

class xArmCartesianController(Node):
    def __init__(self):
        super().__init__('xarm_cartesian_controller')
        
        # 1. 서비스 및 액션 클라이언트 설정
        # 직선 경로 계산 서비스
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        # 계산된 경로 실행 액션 (ExecuteTrajectory가 MoveGroup보다 직접적이고 빠름)
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def move_to_pose(self, x, y, z, roll, pitch, yaw, base_frame, ee_link):
        """
        GUI처럼 작동하는 직선(Cartesian) 이동 함수
        """
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ Cartesian 서비스 서버를 찾을 수 없습니다.')
            return False

        # 1. 목표 포즈(Pose) 구성
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        # RPY -> Quaternion 변환
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]

        # 2. 직선 경로(Cartesian Path) 요청 설정
        req = GetCartesianPath.Request()
        req.header.frame_id = base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.link_name = ee_link
        req.group_name = 'arm'
        req.waypoints = [target_pose] # 현재 위치에서 목표까지의 경유지
        req.max_step = 0.01          # 1cm 간격으로 정밀 경로 생성
        req.jump_threshold = 0.0     # 0.0으로 설정하여 조인트 급변(뒤집힘) 방지

        self.get_logger().info(f'🧩 경로 계산 중: Base={base_frame}, EE={ee_link}...')
        
        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # 3. 경로 유효성 검사 (fraction은 계획 성공 비율)
        if res and res.fraction > 0.9: # 90% 이상 경로가 만들어졌다면
            self.get_logger().info(f'✅ 직선 경로 생성 성공 (진행률: {res.fraction*100:.1f}%)')
            return self._execute_trajectory(res.solution)
        else:
            self.get_logger().error(f'❌ 직선 경로 생성 실패 (진행률: {res.fraction*100:.1f}%)')
            self.get_logger().warn('이유: 목표가 너무 멀거나 충돌 가능성, 혹은 조인트 리미트에 걸렸을 수 있습니다.')
            return False

    def _execute_trajectory(self, trajectory_msg):
        """계산된 궤적을 실제로 로봇에 전송하여 실행합니다."""
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ 실행 액션 서버를 찾을 수 없습니다.')
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self.get_logger().info('🦾 로봇 이동 시작...')
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 실행 요청이 거절되었습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        if result_future.result().status == 4: # SUCCEEDED
            self.get_logger().info('✅ 이동 완료!')
            return True
        else:
            self.get_logger().error('❌ 이동 도중 에러 발생.')
            return False

def main():
    # 사용법: python3 arm_linear_client.py 0.2 0.0 -0.134 -3.1416 0.0 0.0 link_base tool_tcp_link
    if len(sys.argv) < 9:
        print("Usage: python3 script.py <x> <y> <z> <R> <P> <Y> <base_frame> <ee_link>")
        return

    try:
        args = sys.argv[1:7]
        x, y, z, roll, pitch, yaw = [float(a) for a in args]
        base_frame = sys.argv[7]
        ee_link = sys.argv[8]
    except Exception as e:
        print(f"❌ 인자 오류: {e}")
        return

    rclpy.init()
    node = xArmCartesianController()
    
    # 실행
    success = node.move_to_pose(x, y, z, roll, pitch, yaw, base_frame, ee_link)
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()