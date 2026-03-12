import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# MoveIt (원청) 관련 메시지
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# ros2_control (하청) 관련 메시지
from control_msgs.action import FollowJointTrajectory

class HybridRobotController(Node):
    def __init__(self):
        super().__init__('hybrid_robot_controller')
        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/xarm7_traj_controller/follow_joint_trajectory')
        
        # 🚧 가상 장애물을 퍼블리싱할 토픽 생성
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)


    def spawn_virtual_obstacle(self):
        # 1초 정도 기다려야 퍼블리셔가 MoveIt과 완벽히 연결됩니다.
        time.sleep(1.0) 
        
        box_msg = CollisionObject()
        box_msg.id = "virtual_obj"
        # ❗ 로봇의 최하단 고정축 이름을 적어주세요 (예: 'base_link', 'link_base' 등)
        box_msg.header.frame_id = "link_base" 

        # 박스 모양과 크기 설정 (x: 10cm, y: 40cm, z: 40cm짜리 벽)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.3, 0.1, 0.35] 

        # 박스의 중심 좌표 설정 (로봇 정면 30cm 앞, 높이 20cm 지점에 세움)
        pose = Pose()
        pose.position.x = 0.30 
        pose.position.y = 0.00
        pose.position.z = 0.00
        # 회전(Quaternion)은 기본값
        pose.orientation.w = 1.0 

        box_msg.primitives.append(primitive)
        box_msg.primitive_poses.append(pose)
        
        # 동작 지시: 맵에 추가하라(ADD)
        box_msg.operation = CollisionObject.ADD

        self.collision_pub.publish(box_msg)
        self.get_logger().info('🚧 [장애물 생성] 가상의 벽을 로봇 앞(x=0.3)에 세웠습니다!')

    def get_plan_from_moveit(self, target_angle):
        self.get_logger().info('🧠 MoveIt에 궤적 계산을 요청합니다...')
        self.moveit_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm' # ❗ 본인의 그룹명
        
        # ---------------------------------------------------------
        # 🐢 바로 여기서 속도와 가속도를 제어합니다! (0.0 ~ 1.0 사이의 비율)
        # ---------------------------------------------------------
        # 0.2로 설정하면 로봇이 낼 수 있는 최대 속도의 20%로 부드럽게 움직입니다.
        req.max_velocity_scaling_factor = 0.3
        
        # 가속도(출발/정지 시 얼마나 빨리 속도를 올릴지)도 함께 낮춰야 
        # 로봇이 덜컹거리지 않고 묵직하게 움직입니다.
        req.max_acceleration_scaling_factor = 0.5
        # ---------------------------------------------------------


        # 제약 조건 설정 (이전과 동일)
        goal_constraints = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        target_positions = [target_angle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for name, pos in zip(joint_names, target_positions):
            jc = JointConstraint(joint_name=name, position=float(pos), tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            goal_constraints.joint_constraints.append(jc)
        req.goal_constraints.append(goal_constraints)
        goal_msg.request = req

        # 🌟 핵심: 움직이지 말고 계획(배열)만 던져라!
        goal_msg.planning_options.plan_only = True 

        # 전송 및 결과 받기
        future = self.moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('✅ 완벽한 궤적(Waypoints) 생성 완료!')
            # 🌟 MoveIt이 만들어준 완벽한 궤적 배열을 통째로 반환!
            return result.planned_trajectory.joint_trajectory
        else:
            return None

    def execute_and_wait(self, trajectory):
        self.get_logger().info('🦾 컨트롤러에 직접 실행 명령을 내립니다!')
        self.traj_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory # MoveIt이 준 배열을 그대로 집어넣음

        future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        result_future = goal_handle.get_result_async()
        
        try:
            # 이동이 끝날 때까지 대기
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info('🏁 목표 지점 도착 성공!')
            return True
            
        except KeyboardInterrupt:
            # 🚨 Ctrl+C 입력 시 하위 제어기에 직접 Cancel 꽂아버리기! (즉각 정지)
            self.get_logger().warn('\n🛑 [긴급 정지] 하위 제어기 모터 브레이크 가동!')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info('✅ 로봇 급정지 완료.')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = HybridRobotController()
    
    # 왕복 시작 전에 장애물부터 세웁니다.
    node.spawn_virtual_obstacle()
    time.sleep(1.0) # 맵에 반영될 시간 확보

    targets = [1.2, -1.2]
    
    loop = 10


    try:
        for i in range(loop):
            for target in targets:
                # 1. 계산 (Plan)
                planned_traj = node.get_plan_from_moveit(target)
                
                if planned_traj:
                    # 2. 실행 (Execute) - 여기서 멈추고 싶으면 Ctrl+C!
                    success = node.execute_and_wait(planned_traj)
                    if not success:
                        break
                time.sleep(1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()