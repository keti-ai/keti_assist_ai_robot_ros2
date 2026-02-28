import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

class HeadLoopClient(Node):
    def __init__(self):
        super().__init__('head_loop_client')
        # 헤드 컨트롤러의 액션 서버 이름으로 변경
        self._client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def send_goal_and_wait(self, target_pos):
        self._client.wait_for_server()

        # 1. 목표(Goal) 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        # 조인트 이름을 헤드에 맞게 변경
        goal_msg.trajectory.joint_names = ['head_joint1', 'head_joint2']
        
        point = JointTrajectoryPoint()
        # 전달받은 리스트 값을 각각 대입
        point.positions = [target_pos[0], target_pos[1]]
        
        # 🚀 2.8 라디안(1.4 -> -1.4) 이동이 있으므로 여유롭게 4초 부여
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'🚀 목표 위치 [{target_pos[0]} rad, {target_pos[1]} rad]로 이동 명령 전송 중...')
        
        # 2. 목표 전송 및 수락 대기
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 명령이 거절되었습니다.')
            return False

        # 3. 결과 대기 및 예외 처리
        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
            action_response = result_future.result()
            status = action_response.status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'✅ [{target_pos[0]}, {target_pos[1]}] 이동 완료!\n')
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
    node = HeadLoopClient()
    
    # 터미널에서 반복 횟수를 입력받습니다. (기본값: 3회 왕복)
    loop_count = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    # 왕복할 타겟 각도 리스트 (head_joint1, head_joint2)
    targets = [
        [1.4, -0.7],
        [-1.4, 0.3]
    ]
    
    node.get_logger().info(f'=== 총 {loop_count}회 헤드 왕복 테스트를 시작합니다 ===')
    
    success = True
    try:
        for i in range(loop_count):
            node.get_logger().info(f'--- [헤드 왕복 {i+1}/{loop_count} 회차 시작] ---')
            
            for target in targets:
                # 목표 지점으로 이동
                success = node.send_goal_and_wait(target)
                
                # 만약 이동 중 에러가 나면 즉시 전체 루프를 중단합니다.
                if not success:
                    node.get_logger().error('🚨 동작 중 치명적 에러 발생! 안전을 위해 루프를 강제 종료합니다.')
                    break
                
                # 관절에 무리가 가지 않도록 도착 후 0.5초 대기 (안정화)
                time.sleep(0.5)
            
            # 내부 루프에서 에러(break)가 났다면 외부 루프도 종료
            if not success:
                break
                
        if success:
            node.get_logger().info('🎉 모든 헤드 왕복 테스트가 성공적으로 완료되었습니다!')

    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 테스트가 중단되었습니다.')

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()