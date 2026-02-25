import sys
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

class ArmLoopClient(Node):
    def __init__(self):
        super().__init__('arm_loop_interpolated_client')
        self._client = ActionClient(self, FollowJointTrajectory, '/xarm7_traj_controller/follow_joint_trajectory')

    def send_goal_and_wait(self, start_pos, target_pos, duration=4.0, num_steps=40):
        self._client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        self.get_logger().info(f'🚀 [{start_pos} rad] -> [{target_pos} rad] 이동 궤적 생성 중... (Waypoints: {num_steps}개)')

        # ---------------------------------------------------------
        # 🌟 핵심 로직: 위치와 시간을 num_steps 개수만큼 잘게 쪼갭니다.
        # ---------------------------------------------------------
        positions = np.linspace(start_pos, target_pos, num_steps)
        # 시간은 0초가 아닌 (총시간/단계수) 부터 시작하여 점진적으로 증가하도록 합니다.
        times = np.linspace(duration / num_steps, duration, num_steps)

        for pos, t in zip(positions, times):
            point = JointTrajectoryPoint()
            # joint1만 쪼개진 위치를 넣고, 나머지는 0.0으로 고정
            point.positions = [pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 쪼개진 시간을 sec와 nanosec로 분리하여 할당
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            
            goal_msg.trajectory.points.append(point)
        # ---------------------------------------------------------

        self.get_logger().info('명령 전송 및 액션 서버 응답 대기 중...')
        
        # 목표 전송 및 수락 대기
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 명령이 거절되었습니다.')
            return False

        # 결과 대기 및 예외 처리
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
            action_response = result_future.result()
            status = action_response.status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'✅ [{target_pos} rad] 이동 완료!\n')
                return True
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('⚠️ 이동 실패 (Aborted). 오차 초과 또는 충돌 발생!\n')
                return False
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('🛑 이동이 취소되었습니다.\n')
                return False
            else:
                self.get_logger().warn(f'알 수 없는 상태: {status}\n')
                return False
                
        except KeyboardInterrupt:
            self.get_logger().warn('\n강제 종료 감지! 컨트롤러에 취소(Cancel) 명령을 보냅니다...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info('취소 완료. 로봇이 멈춥니다.')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ArmLoopClient()
    
    # 터미널에서 반복 횟수를 입력받습니다. (기본값: 3회)
    loop_count = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    # 타겟 각도 리스트
    targets = [1.2, -1.2]
    
    # 시작할 때 로봇이 0.0에 있다고 가정하고 첫 start_pos를 0.0으로 세팅합니다.
    current_pos = 0.0 
    
    node.get_logger().info(f'=== 총 {loop_count}회 고해상도 왕복 테스트를 시작합니다 ===')
    
    success = True
    try:
        for i in range(loop_count):
            node.get_logger().info(f'--- [왕복 {i+1}/{loop_count} 회차 시작] ---')
            
            for target in targets:
                # 쪼개진 궤적을 전송 (시작 위치와 목표 위치를 모두 전달)
                success = node.send_goal_and_wait(start_pos=current_pos, target_pos=target, duration=4.0, num_steps=40)
                
                if not success:
                    node.get_logger().error('🚨 동작 중 치명적 에러 발생! 안전을 위해 루프를 강제 종료합니다.')
                    break
                
                # 이동이 성공하면 현재 위치를 업데이트합니다.
                current_pos = target
                
                # 관절 보호를 위한 안정화 대기
                time.sleep(0.5)
            
            if not success:
                break
                
        if success:
            node.get_logger().info('🎉 모든 궤적 보간 왕복 테스트가 부드럽게 완료되었습니다!')

    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 테스트가 중단되었습니다.')

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()