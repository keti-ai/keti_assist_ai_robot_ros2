import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class LiftSingleClient(Node):
    def __init__(self):
        super().__init__('lift_single_client')
        self._client = ActionClient(self, FollowJointTrajectory, '/lift_controller/follow_joint_trajectory')

    def send_goal_and_wait(self, target_pos):
        self.get_logger().info('액션 서버 대기 중...')
        self._client.wait_for_server()

        # 1. 목표(Goal) 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [target_pos]
        point.time_from_start.sec = 2  # 2초 동안 부드럽게 이동
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'목표 위치 [{target_pos}m] 전송 중...')
        
        # 2. 목표 전송 및 수락 대기
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('명령이 거절되었습니다 (한계치 초과 등).')
            return

        self.get_logger().info('명령 수락됨! 이동을 시작합니다... (취소하려면 Ctrl+C)')
        
        # 3. 결과 대기 및 Ctrl+C(강제 종료) 감지
        result_future = goal_handle.get_result_async()
        try:
            # 이동이 끝날 때까지 여기서 대기
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info('이동 완료! 프로그램을 종료합니다.')
            
        except KeyboardInterrupt:
            # 4. 사용자가 도중에 Ctrl+C를 눌렀을 때의 처리 (Cancel 전송)
            self.get_logger().warn('\n강제 종료 감지! 컨트롤러에 취소(Cancel) 명령을 보냅니다...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info('취소 완료. 로봇이 멈춥니다.')

def main(args=None):
    rclpy.init(args=args)
    
    # 터미널 실행 시 뒤에 적은 숫자를 가져옴 (없으면 0.3m)
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    
    node = LiftSingleClient()
    node.send_goal_and_wait(target)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()