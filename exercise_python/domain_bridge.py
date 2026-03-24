import rclpy
from rclpy.node import Node
from rclpy.context import Context
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster # TF 발행용
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist # 추가

class SlamwareBridge(Node):
    def __init__(self, slamware_context, main_context):
        # 1. 메인 도메인(0) 노드 초기화
        super().__init__('slamware_bridge_pub', context=main_context)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2. Slamware 도메인(10) 전용 노드 생성
        self.slam_sub_node = Node('slamware_bridge_sub', context=slamware_context)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # 🌟 중요: Slamware가 VOLATILE이므로 똑같이 맞춰야 합니다.
            durability=DurabilityPolicy.VOLATILE 
        )
        # --- [리매핑 설정: Map] ---
        self.map_pub = self.create_publisher(OccupancyGrid, '/slamware_ros_sdk_server_node/map', map_qos)
        self.map_sub = self.slam_sub_node.create_subscription(
            OccupancyGrid, 
            '/slamware_ros_sdk_server_node/map', # 원본 토픽명
            self.map_callback, 
            map_qos
        )

        laser_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # 🌟 중요: Slamware가 VOLATILE이므로 똑같이 맞춰야 합니다.
            durability=DurabilityPolicy.VOLATILE 
        )
        self.laser_pub = self.create_publisher(LaserScan, '/slamware_ros_sdk_server_node/scan', laser_qos)
        self.laser_sub = self.slam_sub_node.create_subscription(
            LaserScan,
            '/slamware_ros_sdk_server_node/scan',
            self.laser_callback,
            laser_qos
        )

        # --- [역방향 브릿지: cmd_vel 제어] ---
        # 메인 도메인(30)에서 명령을 구독
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Slamware 도메인(35)으로 명령을 발행
        self.cmd_vel_pub = self.slam_sub_node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )


        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE 
        )

        # --- [로봇 위치(Pose) 구독 및 TF 변환] ---
        self.pose_sub = self.slam_sub_node.create_subscription(
            PoseStamped,
            '/robot_pose', # Slamware가 주는 토픽
            self.pose_callback,
            volatile_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose', # 메인 도메인(30)에서 RViz가 쏘는 토픽
            self.goal_callback,
            10
        )
        self.goal_pub = self.slam_sub_node.create_publisher(
            PoseStamped,
            '/move_base_simple/goal', # Slamware 도메인(35)이 기다리는 토픽
            10
        )

        self.get_logger().info('✅ Slamware Domain Bridge (35 -> 30) Started')

    def map_callback(self, msg):
        # 메시지 헤더의 frame_id를 메인 시스템 규격에 맞게 수정 가능
        #msg.header.frame_id = 'map'
        self.map_pub.publish(msg)

    def laser_callback(self, msg):
        msg.header.frame_id = 'S2RPLidar_frame'
        self.laser_pub.publish(msg)

    def pose_callback(self, msg):
        """
        Slamware의 PoseStamped 데이터를 메인 시스템의 TF로 발행
        """
        t = TransformStamped()

        # 1. 헤더 설정 (Slamware 맵 프레임 -> 메인 로봇 프레임)
        t.header.stamp = self.get_clock().now().to_msg() # 시간 동기화
        t.header.frame_id = 'slamware_map' # 부모 프레임
        t.child_frame_id = 'base_footprint' # 자식 프레임 (URDF와 일치)

        # 2. 위치 데이터 복사 (Pose -> Transform)
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # 3. 회전 데이터 복사 (Quaternion)
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # 4. 메인 도메인(30)으로 TF 발행
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg):
        """
        메인 시스템(30)의 속도 명령을 Slamware 로봇(35) 도메인으로 그대로 전달
        """
        # 별도의 가공 없이 바로 발행
        self.cmd_vel_pub.publish(msg)

    def goal_callback(self, msg):
        # 프레임 이름을 Slamware가 이해하는 slamware_map으로 변경
        msg.header.frame_id = 'slamware_map'
        self.goal_pub.publish(msg)

def main():
    # 1. 도메인 35 (Slamware 측) 컨텍스트 생성
    slam_context = rclpy.context.Context()
    slam_context.init(domain_id=35)

    # 2. 도메인 30 (메인 시스템 측) 컨텍스트 생성
    # 이 컨텍스트를 익스큐터의 메인으로 사용합니다.
    main_context = rclpy.context.Context()
    main_context.init()

    try:
        # 노드 생성 시 각각의 컨텍스트 주입
        bridge_node = SlamwareBridge(slam_context, main_context)
        
        # 🌟 핵심 수정 부분: Executor에 main_context를 명시적으로 전달
        executor = rclpy.executors.MultiThreadedExecutor(context=main_context)
        
        executor.add_node(bridge_node)
        executor.add_node(bridge_node.slam_sub_node)
        
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 처리
        bridge_node.destroy_node()
        bridge_node.slam_sub_node.destroy_node()
        # rclcpp와 달리 rclpy는 shutdown 시 context를 명시해야 합니다.
        if slam_context.ok():
            slam_context.shutdown()
        if main_context.ok():
            main_context.shutdown()

if __name__ == '__main__':
    main()