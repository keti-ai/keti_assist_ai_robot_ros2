import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# 중요: GripperActionController는 이 메시지 타입을 사용합니다.
from control_msgs.action import GripperCommand

class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        # 설정파일의 'tool_controller' 이름과 일치하도록 경로 수정
        self._client = ActionClient(self, GripperCommand, '/tool_controller/gripper_cmd')

    def send_goal(self, position, effort=50.0):
        self.get_logger().info('그리퍼 액션 서버 대기 중...')
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('서버를 찾을 수 없습니다. 컨트롤러가 실행 중인지 확인하세요.')
            return

        # 1. GripperCommand 목표 생성
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position    # 목표 벌림 정도 (단위: m)
        goal_msg.command.max_effort = effort    # 최대 힘 (설정값: 50.0)

        self.get_logger().info(f'목표: 위치 {position}m, 최대 힘 {effort}N 전송 중...')

        # 2. 목표 전송
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('명령이 거절되었습니다.')
            return

        self.get_logger().info('명령 수락됨! 그리퍼 이동 시작.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # GripperCommand는 결과로 현재 위치, 힘, 도달 여부(reached_goal), 멈춤 여부(stalled)를 줍니다.
        self.get_logger().info(f'이동 완료! 결과 위치: {result.position:.4f}m, Stalled: {result.stalled}')
        # 작업 완료 후 노드 종료를 위해 타이머나 플래그를 쓸 수 있습니다.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # 사용법: ros2 run <pkg> <node> 0.05 (숫자 미입력 시 0.0)
    target_pos = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    
    client = GripperActionClient()
    client.send_goal(target_pos)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()