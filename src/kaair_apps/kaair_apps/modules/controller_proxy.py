from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from rclpy.task import Future

from kaair_apps.utils import ActionState

from typing import TYPE_CHECKING, List



# VSCode 인텔리센스를 위한 타입 힌트 (실행 시에는 무시됨)
if TYPE_CHECKING:
    from ..kaair_api import KaairRobotAPI

class ControllerProxy:
    def __init__(self, api: 'KaairRobotAPI', config: dict):
        self.node = api
        
        # --- Head 상태 관리 변수 (통합 및 정리) ---
        self._head_act = ActionState("Head")
        self._gripper_act = ActionState("Gripper")
        
        
        self._head_goal_fut = None      # 서버 수락 대기용
        self._head_goal_handle = None   # [수정] 실제 취소 명령용 (AttributeError 해결)
        self._head_user_future = None   # 메인문에 반환할 최종 Future
        self._head_is_running = False
        
        # --- 클라이언트 설정 ---
        gripper_cfg = config.get('gripper', {})
        self.tool_client = ActionClient(self.node, GripperCommand, gripper_cfg.get('action', '/tool_controller/gripper_cmd'))

        head_cfg = config.get('head', {})
        self.head_client = ActionClient(self.node, FollowJointTrajectory, head_cfg.get('action', '/head_controller/follow_joint_trajectory'))
        self.head_joints = head_cfg.get('joints', ['head_joint1', 'head_joint2'])


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
    def set_head(self, positions: List[float], duration_sec: float = 2.0) -> Future:
        
        self._head_user_future = Future()
        
        wait_count =1 
        while not self.head_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self._head_user_future.set_result(None)
                self.node.get_logger().warning("Head action server not available")
                return self._head_user_future
            wait_count += 1
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.head_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        goal_msg.trajectory.points.append(point)
        
        self._head_is_running = True
        self._head_status = GoalStatus.STATUS_EXECUTING
        
        self._head_goal_fut = self.head_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._head_feedback_cb)
        
        # 1. 서버 수락(Response) 콜백 연결
        self._head_goal_fut.add_done_callback(self._head_goal_response_cb)
        return self._head_user_future
        
    def _head_feedback_cb(self, feedback_msg):
        """
        궤적 추적 중 실시간으로 호출되는 콜백 함수
        """
        # feedback_msg.feedback 안에 실제 데이터가 들어있습니다.
        feedback: FollowJointTrajectory.Feedback = feedback_msg.feedback
        
        # 1. 현재 조인트 위치 및 오차 데이터 업데이트
        self._head_actual_positions = feedback.actual.positions
        self._head_error_positions = feedback.error.positions
        self._head_feedback_time = self.node.get_clock().now()

        # 2. 로그 출력 (디버깅용 - 너무 자주 찍히면 주석 처리하세요)
        # 조인트 이름을 함께 출력하면 가독성이 좋습니다.
        log_str = "[Head FB] "
        for name, pos, err in zip(self.head_joints, self._head_actual_positions, self._head_error_positions):
            log_str += f"{name}: {pos:.3f}(err:{err:.3f}) "
        self.node.get_logger().info(log_str)
        
    def _head_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[Head] 목표 거절됨")
            self._head_user_future.set_result(None) # 거절 시 완료 처리
            self._head_is_running = False
            self._head_status = GoalStatus.STATUS_REJECTED
            return

        self.node.get_logger().info("[Head] Action Accepted")
        
        # 2. 서버가 수락했다면, 최종 결과를 기다리는 Future를 생성하고 콜백 등록
        self._head_result_fut = goal_handle.get_result_async()
        self._head_result_fut.add_done_callback(self._head_action_result_cb)
        
    def _head_action_result_cb(self, future):
        """최종 동작이 완료(성공/실패/취소)되었을 때 호출되는 콜백"""
        result_wrapped = future.result()
        self._head_status = result_wrapped.status
        self._head_is_running = False # 이제 동작이 끝남

        self._head_user_future.set_result(result_wrapped)

        if self._head_status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("[Head] 목표 지점 도달 완료!")
        elif self._head_status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().warn("[Head] 동작이 취소되었습니다.")
        else:
            self.node.get_logger().error(f"[Head] 동작 실패 (Status: {self._head_status})")
    
    def cancel_head(self):
        """
        외부에서 호출하여 현재 진행 중인 헤드 동작을 취소합니다.
        """
        if self._head_goal_handle is not None:
            self.node.get_logger().warn("🛑 [Head] 취소 요청을 서버에 보냅니다.")
            # 서버에 비동기 취소 요청 전송
            self._head_goal_handle.cancel_goal_async()
            # 취소 후에는 핸들 초기화
            self._head_goal_handle = None
        else:
            # 아직 서버가 수락하기 전(핸들이 없는 상태)일 경우
            if self._head_goal_fut is not None and not self._head_goal_fut.done():
                self.node.get_logger().info("⏳ 서버 수락 대기 중입니다. 수락 즉시 취소되도록 예약합니다.")
                self._head_goal_fut.add_done_callback(
                    lambda f: f.result().cancel_goal_async() if f.result().accepted else None
                )
            else:
                self.node.get_logger().error("❌ 취소할 수 있는 활성 상태의 헤드 명령이 없습니다.")
    
    
    # --- 외부(BT 노드 등)에서 상태를 체크하기 위한 유틸리티 함수 ---
    def is_head_moving(self) -> bool:
        """현재 헤드가 움직이는 중인지 확인"""
        return self._head_is_running

    def get_head_move_done(self) -> bool:
        """명령이 끝났는지(성공이든 실패든) 확인"""
        if self._head_result_fut is None:
            return False
        return self._head_result_fut.done()
    
    
    # def set_head(self, positions: List[float], duration_sec: float = 2.0) -> RobotActionHandle:
    #     """
    #     Head 조인트들을 목표 위치로 이동시킵니다.
    #     :param positions: [joint1, joint2] 위치 리스트 (라디안)
    #     :param duration_sec: 도달 목표 시간
    #     """
    #     if not self.head_client.wait_for_server(timeout_sec=0.1):
    #         self.node.get_logger().error("Head 서버 오프라인")
    #         return

    #     goal_msg = FollowJointTrajectory.Goal()
    #     goal_msg.trajectory.joint_names = self.head_joints
        
    #     point = JointTrajectoryPoint()
    #     point.positions = positions
    #     point.time_from_start.sec = int(duration_sec)
    #     point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
    #     goal_msg.trajectory.points = [point]

    #     send_goal_future = self.head_client.send_goal_async(
    #         goal_msg, feedback_callback=self._head_feedback_cb)
    #     send_goal_future.add_done_callback(self._head_goal_response_cb)
    #     self.node.get_logger().info(f"[Head] 명령 전송: {positions}")

    # def _head_goal_response_cb(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.node.get_logger().error("[Head] 목표 거절됨")
    #         return
    #     goal_handle.get_result_async().add_done_callback(self._head_action_result_cb)

    # def _head_feedback_cb(self, feedback_msg):
    #     pass # 헤드 궤적 추적 피드백 처리 필요 시 작성

    # def _head_action_result_cb(self, future):
    #     status = future.result().status
    #     if status == GoalStatus.STATUS_SUCCEEDED:
    #         self.node.get_logger().info("[Head] 이동 완료!")
    #     else:
    #         self.node.get_logger().warn(f"[Head] 동작 중단 (Status: {status})")