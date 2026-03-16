import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation as R

class xArmRelativeController(Node):
    def __init__(self):
        super().__init__('xarm_relative_controller')
        
        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 서비스 및 액션 클라이언트
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def move_relative_tool(self, dx, dy, dz):
        # 🌟 아까 성공했던 정확한 프레임 이름으로 고정
        base_frame = 'link_base'
        ee_link = 'tool_tcp_link'

        self.get_logger().info(f"⏳ TF 데이터 수집을 위해 잠시 대기 중...")
        
        # 🌟 핵심: TF 버퍼가 채워질 때까지 최소 1초는 spin을 돌려줘야 합니다.
        start_wait = self.get_clock().now()
        while (self.get_clock().now() - start_wait).nanoseconds / 1e9 < 1.5:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. 현재 로봇의 Pose를 Base 기준에서 읽어옴
        try:
            self.get_logger().info(f"🔍 현재 위치 파악 중... ({base_frame} -> {ee_link})")
            # lookup_transform 자체에도 timeout을 넉넉히 줍니다.
            trans = self.tf_buffer.lookup_transform(
                base_frame, 
                ee_link, 
                rclpy.time.Time(), 
                rclpy.duration.Duration(seconds=3.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ 현재 위치를 읽을 수 없습니다: {e}")
            self.get_logger().warn("Tip: 로봇 드라이버가 켜져 있는지, 'ros2 run tf2_ros tf2_monitor'로 프레임이 보이는지 확인하세요.")
            return False

        # 현재 위치와 자세 추출
        cur_p = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        cur_q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        
        # 2. Tool 좌표계 기준 이동량을 Base 좌표계 기준으로 변환
        rot_matrix = R.from_quat(cur_q).as_matrix()
        relative_vector_tool = np.array([dx, dy, dz])
        relative_vector_base = rot_matrix @ relative_vector_tool
        
        # 3. 최종 목표 Pose 계산
        target_p = cur_p + relative_vector_base
        
        target_pose = Pose()
        target_pose.position.x = target_p[0]
        target_pose.position.y = target_p[1]
        target_pose.position.z = target_p[2]
        target_pose.orientation.x = cur_q[0]
        target_pose.orientation.y = cur_q[1]
        target_pose.orientation.z = cur_q[2]
        target_pose.orientation.w = cur_q[3]

        self.get_logger().info(f"🚀 상대 이동 계산 완료 (Base 기준 목표): XYZ({target_p[0]:.4f}, {target_p[1]:.4f}, {target_p[2]:.4f})")

        # 4. Cartesian Path 요청
        req = GetCartesianPath.Request()
        req.header.frame_id = base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.link_name = ee_link
        req.group_name = 'arm'
        req.waypoints = [target_pose]
        req.max_step = 0.005 # 5mm 간격으로 더 정밀하게 생성
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ MoveIt 서비스를 찾을 수 없습니다.")
            return False

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.fraction > 0.9:
            self.get_logger().info(f"✅ 직선 경로 생성 성공 ({res.fraction*100:.1f}%)")
            # 🌟 수정: .joint_trajectory를 붙이지 않고 res.solution을 통째로 보냅니다.
            return self._execute_trajectory(res.solution)
        else:
            self.get_logger().error(f"❌ 경로 생성 실패 (진행률: {res.fraction*100:.1f}%)")
            return False

    def _execute_trajectory(self, trajectory_msg):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            return False
            
        goal_msg = ExecuteTrajectory.Goal()
        # 🌟 이제 trajectory_msg는 RobotTrajectory 타입이므로 assert 에러가 나지 않습니다.
        goal_msg.trajectory = trajectory_msg
        
        self.get_logger().info("🦾 로봇 이동 시작...")
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == 4

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 arm_toolxyz_client.py <dx> <dy> <dz>")
        return

    dx, dy, dz = map(float, sys.argv[1:4])

    rclpy.init()
    node = xArmRelativeController()
    try:
        node.move_relative_tool(dx, dy, dz)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()