import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes
from control_msgs.action import FollowJointTrajectory

class StrictArmController(Node):
    def __init__(self):
        super().__init__('strict_arm_controller')
        
        # 1. 설정: 조인트 이름 및 개수
        self.JOINT_NAMES = [f'joint{i}' for i in range(1, 8)] # joint1 ~ joint7
        self.TOLERANCE = 0.01  # 도달 인정 오차 (약 0.5도)

        # 상태 모니터링을 위한 변수
        self.current_positions = [None] * 7
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)

        # 액션 클라이언트 설정
        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/xarm7_traj_controller/follow_joint_trajectory')

    def _joint_state_callback(self, msg):
        """실시간 조인트 상태를 업데이트합니다."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                self.current_positions[i] = msg.position[msg.name.index(name)]

    def move_arm(self, target_angles):
        """7개의 조인트 값을 받아 계획, 실행 및 최종 검증을 수행합니다."""
        if len(target_angles) != 7:
            self.get_logger().error(f'❌ 인자 오류: 7개의 조인트 값이 필요합니다. (입력됨: {len(target_angles)}개)')
            return False

        # 1. MoveIt 계획 요청
        planned_traj = self._get_plan(target_angles)
        if not planned_traj:
            self.get_logger().error('❌ 계획 실패: 리미트 초과 또는 장애물 충돌 가능성.')
            return False

        # 2. 하위 컨트롤러 실행
        self.get_logger().info('🦾 xArm 7 이동 시작...')
        execute_success = self._execute_trajectory(planned_traj)

        # 3. [핵심] 최종 위치 정밀 대조
        # MoveIt이 리미트에 맞춰 궤적을 깎아서(Clamping) 실행했는지 확인합니다.
        if execute_success:
            if None not in self.current_positions:
                for i, (target, actual) in enumerate(zip(target_angles, self.current_positions)):
                    diff = abs(target - actual)
                    if diff > self.TOLERANCE:
                        self.get_logger().error(
                            f'❌ 검증 실패: {self.JOINT_NAMES[i]}이 목표에 도달하지 못함. '
                            f'(목표: {target:.4f}, 실제: {actual:.4f}, 차이: {diff:.4f})'
                        )
                        return False # 하나라도 차이가 크면 즉시 실패 리턴
                
                self.get_logger().info('✅ 성공: 모든 조인트가 목표 지점에 정확히 도달했습니다.')
                return True
        return False

    def _get_plan(self, target_angles):
        self.moveit_client.wait_for_server()
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'  # MoveIt 설정 그룹명
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # 7개 조인트 제약 조건 추가
        goal_constraints = Constraints()
        for name, pos in zip(self.JOINT_NAMES, target_angles):
            jc = JointConstraint(joint_name=name, position=float(pos),
                                 tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            goal_constraints.joint_constraints.append(jc)
        
        req.goal_constraints.append(goal_constraints)
        goal_msg.request = req
        goal_msg.planning_options.plan_only = True

        future = self.moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        result = res_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return result.planned_trajectory.joint_trajectory
        return None

    def _execute_trajectory(self, trajectory):
        self.traj_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal(trajectory=trajectory)
        future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == 4 # SUCCEEDED

def main():
    # 7개의 인자를 받음 (예: ros2 run pkg script 0.1 0.2 ... 0.7)
    if len(sys.argv) < 8:
        print("Usage: ros2 run <pkg> <script> <j1> <j2> <j3> <j4> <j5> <j6> <j7>")
        return

    try:
        targets = [float(sys.argv[i]) for i in range(1, 8)]
    except ValueError:
        print("❌ 오류: 모든 인자는 숫자여야 합니다.")
        return

    rclpy.init()
    node = StrictArmController()
    
    success = node.move_arm(targets)
    
    node.destroy_node()
    rclpy.shutdown()
    
    # 최종 리턴 (성공 0, 실패 1)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()