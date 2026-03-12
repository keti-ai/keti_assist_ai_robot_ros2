import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

class HeadDirectController(Node):
    def __init__(self):
        super().__init__('head_direct_controller')
        
        # 설정: 조인트 이름 및 허용 오차
        self.JOINT_NAMES = ['head_joint1', 'head_joint2']
        self.TOLERANCE = 0.05  # 약 2.8도 (헤드는 기어 백래시가 있을 수 있어 약간 여유 있게 설정)

        # 실시간 상태 모니터링
        self.current_positions = [None, None]
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # 컨트롤러 액션 클라이언트
        self._client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def _js_cb(self, msg):
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                self.current_positions[i] = msg.position[msg.name.index(name)]

    def move_head(self, target_pos):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ 헤드 컨트롤러 서버를 찾을 수 없습니다.')
            return False

        # 1. 목표(Goal) 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = [float(target_pos[0]), float(target_pos[1])]
        # 이동 시간 설정 (너무 빠르면 하드웨어에 무리가 가니 거리에 따라 조절하거나 고정값 부여)
        point.time_from_start.sec = 2 
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'🚀 헤드 이동 명령: Pan({target_pos[0]}), Tilt({target_pos[1]})')
        
        # 2. 전송 및 결과 대기
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 헤드 컨트롤러가 명령을 거절했습니다.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            # 3. 실제 도달 여부 검증 (Clamping 확인)
            return self._verify_reach(target_pos)
        else:
            self.get_logger().error(f'⚠️ 이동 실패: 상태 코드 {status}')
            return False

    def _verify_reach(self, targets):
        """현재 위치와 목표 위치를 비교하여 실제 도달 여부를 확인합니다."""
        if None in self.current_positions:
            self.get_logger().warn('⚠️ 현재 조인트 상태를 읽을 수 없어 검증을 건너뜁니다.')
            return True

        for i, (target, actual) in enumerate(zip(targets, self.current_positions)):
            diff = abs(target - actual)
            if diff > self.TOLERANCE:
                self.get_logger().error(
                    f'❌ 검증 실패: {self.JOINT_NAMES[i]} 도달 불가. '
                    f'(목표: {target:.3f}, 실제: {actual:.3f}, 차이: {diff:.3f})'
                )
                return False
        
        self.get_logger().info('✅ 헤드 목표 지점 도달 완료.')
        return True

def main():
    # 2개의 인자를 받음 (Pan, Tilt)
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