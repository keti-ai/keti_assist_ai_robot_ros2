import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

ERROR_CODE_MAP = {
    MoveItErrorCodes.SUCCESS:                  'SUCCESS',
    MoveItErrorCodes.PLANNING_FAILED:          'PLANNING_FAILED',
    MoveItErrorCodes.INVALID_MOTION_PLAN:      'INVALID_MOTION_PLAN',
    MoveItErrorCodes.CONTROL_FAILED:           'CONTROL_FAILED',
    MoveItErrorCodes.NO_IK_SOLUTION:           'NO_IK_SOLUTION',
    MoveItErrorCodes.GOAL_IN_COLLISION:        'GOAL_IN_COLLISION',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
}


class MoveJClient(Node):
    def __init__(self):
        super().__init__('movej_client')
        self._client = ActionClient(self, MoveGroup, 'move_action')

    # ── 내부 헬퍼 ───────────────────────────────────────────────
    def _build_goal(
        self,
        joint_names: list[str],
        target_positions: list[float],
        *,
        group_name: str = 'arm',
        velocity_scaling: float = 0.3,
        accel_scaling: float = 0.3,
        tolerance: float = 0.001,
        plan_only: bool = False,
    ) -> MoveGroup.Goal:
        """MoveGroup 액션 Goal 메시지를 조립해 반환합니다."""
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = accel_scaling

        constraints = Constraints()
        for name, pos in zip(joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = plan_only
        return goal

    def _send_and_wait(self, goal: MoveGroup.Goal) -> MoveGroup.Result | None:
        """Goal을 전송하고 완료될 때까지 블로킹으로 대기한 뒤 결과를 반환합니다."""
        self.get_logger().info('액션 서버 연결 대기 중...')
        self._client.wait_for_server()
        self.get_logger().info('Goal 전송!')

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal 거절됨 (서버가 수락하지 않음)')
            return None

        self.get_logger().info('Goal 수락됨 — 실행 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Goal 상태 이상: status={wrapped.status}')
            return None

        return wrapped.result

    # ── 공개 API ────────────────────────────────────────────────
    def movej(
        self,
        target_positions: list[float],
        joint_names: list[str] | None = None,
        *,
        group_name: str = 'arm',
        velocity_scaling: float = 0.3,
        accel_scaling: float = 0.3,
    ) -> bool:
        """
        지정한 관절 목표 각도(rad)로 로봇을 이동시킵니다.

        Args:
            target_positions: 각 관절의 목표 각도 리스트 [rad]
            joint_names:      관절 이름 리스트 (None이면 전역 JOINT_NAMES 사용)
            group_name:       MoveIt 플래닝 그룹 이름
            velocity_scaling: 최대 속도 배율 (0.0 ~ 1.0)
            accel_scaling:    최대 가속도 배율 (0.0 ~ 1.0)

        Returns:
            True: 이동 성공 / False: 실패
        """
        names = joint_names or JOINT_NAMES
        if len(names) != len(target_positions):
            self.get_logger().error(
                f'관절 이름({len(names)}개)과 목표 각도({len(target_positions)}개) 개수가 다릅니다.'
            )
            return False

        goal = self._build_goal(
            names,
            target_positions,
            group_name=group_name,
            velocity_scaling=velocity_scaling,
            accel_scaling=accel_scaling,
        )

        result = self._send_and_wait(goal)
        if result is None:
            return False

        code = result.error_code.val
        label = ERROR_CODE_MAP.get(code, f'UNKNOWN({code})')

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'✅ movej 성공: {label}')
            return True
        else:
            self.get_logger().error(f'❌ movej 실패: {label}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MoveJClient()

    try:
        # ── 예시 1: 모든 관절을 0도(홈 포지션)로 이동 ──────────────
        node.get_logger().info('=== [1/2] 홈 포지션으로 이동 ===')
        home = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.movej(home, velocity_scaling=0.1, accel_scaling=0.3)

        # ── 예시 2: joint1만 1.2 rad 회전 ───────────────────────────
        node.get_logger().info('=== [2/2] joint1 → 1.2 rad ===')
        target = [1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.movej(target, velocity_scaling=0.1, accel_scaling=0.3)

    except KeyboardInterrupt:
        node.get_logger().warn('사용자 중단 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
