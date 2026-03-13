import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

class HeadDirectController(Node):
    def __init__(self):
        super().__init__('head_direct_controller')
        
        # 설정: 조인트 이름
        self.JOINT_NAMES = ['head_joint1', 'head_joint2']

        # 컨트롤러 액션 클라이언트
        self._client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def move_head(self, target_pos):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ 헤드 컨트롤러 서버를 찾을 수 없습니다.')
            return False

        # 1. 목표(Goal) 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = [float(target_pos[0]), float(target_pos[1])]
        point.time_from_start.sec = 1  # 2초 동안 이동
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'🚀 헤드 이동 시작: Pan({target_pos[0]}), Tilt({target_pos[1]})')
        
        # 2. 전송 및 결과 대기
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 헤드 컨트롤러가 명령을 거부했습니다.')
            return False

        # 3. 액션 결과(Future) 완료까지 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # 4. 상태 코드로 성공 여부 판단
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 헤드 목표 지점 도달 완료 (Action Succeeded)')
            return True
        else:
            self.get_logger().error(f'⚠️ 이동 실패: 액션 상태 코드 {status}')
            return False

def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run <pkg> <script> <pan_angle> <tilt_angle>")
        return

    try:
        targets = [float(sys.argv[1]), float(sys.argv[2])]
    except ValueError:
        print("❌ 오류: 인자는 숫자(radian)여야 합니다.")
        return

    rclpy.init()
    node = HeadDirectController()
    
    success = node.move_head(targets)
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()