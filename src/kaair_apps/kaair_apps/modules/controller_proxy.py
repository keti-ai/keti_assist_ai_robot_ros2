from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from kaair_apps.utils import RobotActionHandle

from typing import TYPE_CHECKING, List


# VSCode 인텔리센스를 위한 타입 힌트 (실행 시에는 무시됨)
if TYPE_CHECKING:
    from ..kaair_api import KaairRobotAPI

class ControllerProxy:
    def __init__(self, api: 'KaairRobotAPI', config: dict):
        """
        ControllerProxy 초기화
        :param api: Main API 객체 (KaairRobotAPI 인스턴스)
        """
        self.node = api  # KaairRobotAPI가 Node를 상속받았으므로 바로 사용


        # 1. 'gripper' 딕셔너리를 먼저 가져온 후, 그 안에서 'action'을 찾습니다.
        gripper_action = config.get('gripper', {}).get('action', '/tool_controller/gripper_cmd')
        self.tool_client = ActionClient(self.node, GripperCommand, gripper_action)

        # 2. 'head' 딕셔너리를 가져온 후, 그 안에서 'action'을 찾습니다.
        head_action = config.get('head', {}).get('action', '/head_controller/follow_joint_trajectory')
        self.head_client = ActionClient(self.node, FollowJointTrajectory, head_action)
        # YAML에서 조인트 이름을 가져오거나 기본값 설정
        self.head_joints = config.get('head', {}).get('joints', ['head_joint1', 'head_joint2'])


    # ==========================================
    # 1. Gripper 제어 로직 (Prefix: _gripper_*)
    # ==========================================
    def set_gripper(self, position: float):
        if not self.tool_client.wait_for_server(timeout_sec=0.1):
            self.node.get_logger().error("Gripper 서버 오프라인")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        
        send_goal_future = self.tool_client.send_goal_async(
            goal_msg, feedback_callback=self._gripper_feedback_cb)
        send_goal_future.add_done_callback(self._gripper_goal_response_cb)
        self.node.get_logger().info(f"[Gripper] 명령 전송: {position}m")

    def _gripper_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[Gripper] 목표 거절됨")
            return
        goal_handle.get_result_async().add_done_callback(self._gripper_action_result_cb)

    def _gripper_feedback_cb(self, feedback_msg):
        pass # 필요한 경우 피드백 처리

    def _gripper_action_result_cb(self, future):
        result = future.result().result
        self.node.get_logger().info(f'[Gripper] 완료! 위치: {result.position:.4f}m, 잡기성공: {result.stalled}')


    # ==========================================
    # 2. Head 제어 로직 (Prefix: _head_*)
    # ==========================================
    def set_head(self, positions: List[float], duration_sec: float = 2.0) -> RobotActionHandle:
        """
        Head 조인트들을 목표 위치로 이동시킵니다.
        :param positions: [joint1, joint2] 위치 리스트 (라디안)
        :param duration_sec: 도달 목표 시간
        """
        if not self.head_client.wait_for_server(timeout_sec=0.1):
            self.node.get_logger().error("Head 서버 오프라인")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.head_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        goal_msg.trajectory.points = [point]

        send_goal_future = self.head_client.send_goal_async(
            goal_msg, feedback_callback=self._head_feedback_cb)
        send_goal_future.add_done_callback(self._head_goal_response_cb)
        self.node.get_logger().info(f"[Head] 명령 전송: {positions}")

    def _head_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[Head] 목표 거절됨")
            return
        goal_handle.get_result_async().add_done_callback(self._head_action_result_cb)

    def _head_feedback_cb(self, feedback_msg):
        pass # 헤드 궤적 추적 피드백 처리 필요 시 작성

    def _head_action_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("[Head] 이동 완료!")
        else:
            self.node.get_logger().warn(f"[Head] 동작 중단 (Status: {status})")