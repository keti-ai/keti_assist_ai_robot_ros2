import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes
from control_msgs.action import FollowJointTrajectory

class StrictLiftController(Node):
    def __init__(self):
        super().__init__('strict_lift_controller')
        
        # [중요] URDF와 동일한 엄격한 리미트 설정
        self.LIFT_JOINT_NAME = 'lift_joint'
        self.LIMIT_MIN = 0.0
        self.LIMIT_MAX = 0.8  # URDF 상의 upper limit과 동일하게 설정

        # 상태 모니터링
        self.current_pos = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/lift_controller/follow_joint_trajectory')

    def _js_cb(self, msg):
        if self.LIFT_JOINT_NAME in msg.name:
            self.current_pos = msg.position[msg.name.index(self.LIFT_JOINT_NAME)]

    def move_lift(self, target):
        self.get_logger().info(f'🔍 검증 시작: 목표 {target}m (제한: {self.LIMIT_MIN} ~ {self.LIMIT_MAX})')

        # 1. 소프트웨어 레벨 차단 (의도적 리미트 초과 체크)
        if target < self.LIMIT_MIN or target > self.LIMIT_MAX:
            self.get_logger().error(f'❌ 실패: 입력값이 URDF 범위를 벗어났습니다. (Target: {target}m)')
            return False

        # 2. MoveIt 계획 요청
        plan = self._get_plan(target)
        if not plan:
            return False

        # 3. 하위 컨트롤러 실행
        self.get_logger().info(f'🦾 실행 중...')
        if not self._execute(plan):
            return False

        # 4. 최종 위치 대조 (Clamping 여부 확인)
        # 만약 MoveIt이 슬그머니 값을 깎아서 이동했다면 여기서 걸러집니다.
        if self.current_pos is not None:
            diff = abs(self.current_pos - target)
            if diff > 0.01: # 1cm 이상 차이나면 실패 처리
                self.get_logger().error(f'❌ 실패: 목표치 도달 불가. 현재위치: {self.current_pos:.3f}m')
                return False
        
        self.get_logger().info('✅ 성공: 목표 지점에 정확히 도달했습니다.')
        return True

    def _get_plan(self, target):
        self.moveit_client.wait_for_server()
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'lift'
        
        jc = JointConstraint(joint_name=self.LIFT_JOINT_NAME, position=float(target),
                             tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
        req.goal_constraints.append(Constraints(joint_constraints=[jc]))
        goal.request = req
        goal.planning_options.plan_only = True

        future = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        res = res_future.result().result
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            return res.planned_trajectory.joint_trajectory
        return None

    def _execute(self, trajectory):
        self.traj_client.wait_for_server()
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == 4 # GoalStatus.STATUS_SUCCEEDED

def main():
    if len(sys.argv) < 2: return
    try:
        val = float(sys.argv[1])
    except: return

    rclpy.init()
    node = StrictLiftController()
    success = node.move_lift(val)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()