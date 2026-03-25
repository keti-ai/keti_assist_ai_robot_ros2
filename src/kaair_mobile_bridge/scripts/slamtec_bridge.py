#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, DriveOnHeading, Spin
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Point
from visualization_msgs.msg import Marker
# 기존 import 아래에 추가
from slamware_ros_sdk.msg import CancelActionRequest, Line2DFlt32Array

from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import argparse
import math
import time
from copy import deepcopy


class SlamwareBridge(Node):
    def __init__(self, slamware_context, main_context):
        super().__init__('mobile_bridge', context=main_context)
        self.cb_group = ReentrantCallbackGroup()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.slam_sub_node = Node('slamware_bridge_sub', context=slamware_context)
        self.current_pose = None
        self.active_goal_handle = None

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self._drive_action_server = ActionServer(
            self, DriveOnHeading, 'drive_on_heading',
            execute_callback=self.execute_drive_callback,
            goal_callback=self.goal_drive_callback,
            cancel_callback=self.cancel_drive_callback,
            callback_group=self.cb_group # 기존과 같은 스레드풀 사용
        )

        # 🌟 상대 각도 회전 (예: 현재 위치에서 90도 좌회전)
        self._spin_rel_server = ActionServer(
            self, Spin, 'spin',
            execute_callback=self.execute_spin_rel_callback,
            cancel_callback=self.cancel_drive_callback,
            callback_group=self.cb_group
        )

        # 🌟 절대 각도 회전 (예: 맵 기준 북쪽(90도) 바라보기)
        self._spin_abs_server = ActionServer(
            self, Spin, 'spin_absolute',
            execute_callback=self.execute_spin_abs_callback,
            cancel_callback=self.cancel_drive_callback,
            callback_group=self.cb_group
        )


        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', volatile_qos)
        self.map_sub = self.slam_sub_node.create_subscription(
            OccupancyGrid, '/slamware_ros_sdk_server_node/map', self.map_callback, volatile_qos)

        self.laser_pub = self.create_publisher(LaserScan, '/scan', volatile_qos)
        self.laser_sub = self.slam_sub_node.create_subscription(
            LaserScan, '/slamware_ros_sdk_server_node/scan', self.laser_callback, volatile_qos)

        self.pose_sub = self.slam_sub_node.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, volatile_qos)

        self.cmd_vel_pub = self.slam_sub_node.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.goal_pub = self.slam_sub_node.create_publisher(
            PoseStamped, '/move_base_simple/goal', 10)
        self.goal_topic_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_topic_callback, 10)

        self.slam_cancel_pub = self.slam_sub_node.create_publisher(
            CancelActionRequest, '/slamware_ros_sdk_server_node/cancel_action', 10)

        # [메인 도메인] RViz2 시각화용 Marker 퍼블리셔 생성
        self.virtual_wall_pub = self.create_publisher(
            Marker, '/virtual_walls_marker', 10)  # 이름 충돌 방지를 위해 _marker 추가 권장

        # [Slamware 도메인] 가상벽 데이터 구독
        self.virtual_wall_sub = self.slam_sub_node.create_subscription(
            Line2DFlt32Array, '/virtual_walls', 
            self.virtual_wall_callback, volatile_qos)


        self.get_logger().info(
            f'✅ Bridge Started: '
            f'[Main Domain: {main_context.get_domain_id()}] <-> '
            f'[Slamware Domain: {slamware_context.get_domain_id()}]'
        )

    def map_callback(self, msg):
        self.map_pub.publish(msg)

    def laser_callback(self, msg):
        msg.header.frame_id = 'S2RPLidar_frame'
        self.laser_pub.publish(msg)

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'slamware_map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_pub.publish(msg)

    def goal_topic_callback(self, msg):
        msg.header.frame_id = 'slamware_map'
        self.goal_pub.publish(msg)

    def goal_callback(self, goal_request):
        self.get_logger().info('📥 Received NavigateToPose goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('🛑 Received cancel request')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    #  실제 네비게이션 로직 - 별도 스레드에서 동기 실행
    # ------------------------------------------------------------------ #
# ------------------------------------------------------------------ #
    #  execute_callback - 일반 동기 함수 (MultiThreadedExecutor가 스레드 자동 할당)
    # ------------------------------------------------------------------ #
    def execute_callback(self, goal_handle):
        # 🌟 새 목표가 들어오면 현재 활성 목표를 이 goal_handle로 덮어씁니다.
        self.active_goal_handle = goal_handle

        self.get_logger().info('🏃 [1] 액션 콜백 진입: 목표 퍼블리시 시작...')

        goal_pose = goal_handle.request.pose
        goal_pose.header.frame_id = 'slamware_map'
        self.goal_pub.publish(goal_pose)

        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        STALL_POS_THRESHOLD = 0.02   
        STALL_YAW_THRESHOLD = 0.05   
        STALL_TIME_LIMIT    = 3.0    
        TOTAL_TIMEOUT       = 180.0  
        ARRIVAL_DISTANCE    = 0.25   
        ARRIVAL_ANGLE       = 0.15   
        LOOP_INTERVAL       = 0.5

        goal_yaw = self.get_yaw(goal_pose.pose.orientation)

        last_snapshot = None
        last_move_time = self.get_clock().now()
        start_time = self.get_clock().now()

        while True:
            # 🌟 [A-1] 탈출구 0: 새로운 목표가 들어온 경우 (Preemption)
            if self.active_goal_handle != goal_handle:
                self.get_logger().warn('🔄 [Preempt] 새 목표 수신. 기존 감시를 부드럽게 종료합니다.')
                # RViz2 튕김 방지 1: abort() 대신 canceled() 사용
                goal_handle.canceled() 
                # RViz2 튕김 방지 2: C++ 클라이언트가 상태를 받을 수 있도록 아주 잠깐 대기
                time.sleep(0.1) 
                return result

            # 🌟 [A-2] 탈출구 1: 클라이언트 취소 요청 (Cancel 버튼)
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('🛑 [Cancel] 클라이언트에 의해 취소됨')
                
                from slamware_ros_sdk.msg import CancelActionRequest
                self.slam_cancel_pub.publish(CancelActionRequest())
                
                goal_handle.canceled()
                # RViz2 튕김 방지 2: 취소 상태가 DDS 통신망을 타고 RViz2에 도착할 시간을 줍니다.
                time.sleep(0.1) 
                return result

            now = self.get_clock().now()
            elapsed_total = (now - start_time).nanoseconds / 1e9

            # [B] 탈출구 2: 타임아웃
            if elapsed_total > TOTAL_TIMEOUT:
                self.get_logger().error(f'⏳ [Timeout] 전체 제한시간 초과')
                goal_handle.abort()
                return result

            if self.current_pose is None:
                time.sleep(LOOP_INTERVAL)
                continue

            current_snapshot = deepcopy(self.current_pose)
            current_yaw = self.get_yaw(current_snapshot.orientation)

            # --- 위치 및 각도 오차 계산 ---
            dx = goal_pose.pose.position.x - current_snapshot.position.x
            dy = goal_pose.pose.position.y - current_snapshot.position.y
            dist_to_goal = math.sqrt(dx**2 + dy**2)
            
            # 🌟 [180도 회전 허용 로직]
            raw_yaw_error = abs(self.normalize_angle(goal_yaw - current_yaw))
            
            # 오차가 90도(pi/2) 이상 나면, 반대쪽(180도 뒤집힌 상태)으로 계산
            # 즉, 로봇이 앞을 보든 뒤를 보든 일직선상으로 정렬되면 오차를 0에 가깝게 만듭니다.
            if raw_yaw_error > (math.pi / 2.0):
                yaw_error = abs(math.pi - raw_yaw_error)
            else:
                yaw_error = raw_yaw_error

            feedback_msg.distance_remaining = dist_to_goal
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'📍 거리: {dist_to_goal:.2f}m | 각도 오차: {math.degrees(yaw_error):.1f}° | '
                f'정지 시간: {(now - last_move_time).nanoseconds / 1e9:.1f}s',
                throttle_duration_sec=1.0
            )

            # [D] 정지(Stall) 체크 (위치 & 회전)
            if last_snapshot is not None:
                move_delta = math.sqrt(
                    (current_snapshot.position.x - last_snapshot.position.x)**2 +
                    (current_snapshot.position.y - last_snapshot.position.y)**2
                )
                last_yaw = self.get_yaw(last_snapshot.orientation)
                yaw_delta = abs(self.normalize_angle(current_yaw - last_yaw))

                if move_delta > STALL_POS_THRESHOLD or yaw_delta > STALL_YAW_THRESHOLD:
                    last_move_time = now

            last_snapshot = current_snapshot

            # [E] 정지 상태 초과 시 최종 판별
            stall_duration = (now - last_move_time).nanoseconds / 1e9
            if stall_duration > STALL_TIME_LIMIT:
                if dist_to_goal < ARRIVAL_DISTANCE and yaw_error < ARRIVAL_ANGLE:
                    self.get_logger().info(f'🏁 [도착 성공] 오차: {dist_to_goal:.2f}m, {math.degrees(yaw_error):.1f}°')
                    goal_handle.succeed()
                else:
                    self.get_logger().error(f'⚠️ [막힘/고립] 남은 거리: {dist_to_goal:.2f}m, 각도 오차: {math.degrees(yaw_error):.1f}°')
                    goal_handle.abort()
                return result

            time.sleep(LOOP_INTERVAL)


    def goal_drive_callback(self, goal_request):
        self.get_logger().info('📥 [Drive] 전/후진 이동 명령 수신됨')
        return GoalResponse.ACCEPT

    def cancel_drive_callback(self, goal_handle):
        self.get_logger().info('🛑 [Drive] 전/후진 취소 명령 수신됨')
        return CancelResponse.ACCEPT


    def execute_drive_callback(self, goal_handle):
        # 1. 목표 거리와 방향 설정
        target_x = goal_handle.request.target.x
        direction_multiplier = 1.0 if target_x >= 0 else -1.0
        target_distance = abs(target_x)
        
        # 🌟 속도는 철저하게 "절댓값"으로만 취급합니다.
        base_speed = abs(goal_handle.request.speed)

        direction_str = "전진" if direction_multiplier > 0 else "후진"
        self.get_logger().info(f'🚗 [{direction_str}] 목표 거리: {target_distance}m, 기준 속도: {base_speed}m/s')

        while self.current_pose is None:
            time.sleep(0.1)

        start_pose = deepcopy(self.current_pose)
        start_yaw = self.get_yaw(start_pose.orientation)

        cmd_vel = Twist()
        feedback_msg = DriveOnHeading.Feedback()
        result = DriveOnHeading.Result()

        LOOP_INTERVAL = 0.05  # 20Hz
        K_p = 2.5             # 회전 보정 게인
        DECEL_DIST = 0.3      # 30cm 전부터 감속
        MIN_SPEED = 0.08      # 최소 0.05m/s (절댓값)

        while True:
            # [A] 취소 요청
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'🛑 [{direction_str}] 주행 취소됨')
                self.cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                return result

            if self.current_pose is None:
                time.sleep(LOOP_INTERVAL)
                continue

            # 2. 이동 거리 및 남은 거리 계산
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            dist_traveled = math.sqrt(
                (curr_x - start_pose.position.x)**2 + 
                (curr_y - start_pose.position.y)**2
            )
            remaining_dist = target_distance - dist_traveled

            feedback_msg.distance_traveled = dist_traveled
            goal_handle.publish_feedback(feedback_msg)

            # [B] 목표 거리 도달 판별
            if remaining_dist <= 0.0:
                self.get_logger().info(f'🏁 [{direction_str}] 완료! 실제 이동 거리: {dist_traveled:.3f}m')
                self.cmd_vel_pub.publish(Twist()) # 정지
                goal_handle.succeed()
                return result

            # 3. 감속 로직 (절댓값 기반으로만 계산)
            if remaining_dist < DECEL_DIST:
                decel_ratio = remaining_dist / DECEL_DIST
                dynamic_speed = base_speed * decel_ratio
                
                # 최소 속도 방어선 (멈춤 방지)
                if dynamic_speed < MIN_SPEED:
                    dynamic_speed = MIN_SPEED
            else:
                dynamic_speed = base_speed

            # 4. 🌟 최종 적용 (여기서 단 한 번만 방향 부호를 곱해줍니다!)
            cmd_vel.linear.x = dynamic_speed * direction_multiplier

            # 5. 각도 슬립 보정
            curr_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(start_yaw - curr_yaw)
            
            # 차동 구동 로봇: 틀어진 각도의 반대 방향으로 회전력 부여
            cmd_vel.angular.z = K_p * yaw_error

            # 최대 각속도 제한
            MAX_ANGULAR_VEL = 0.5 
            if cmd_vel.angular.z > MAX_ANGULAR_VEL:
                cmd_vel.angular.z = MAX_ANGULAR_VEL
            elif cmd_vel.angular.z < -MAX_ANGULAR_VEL:
                cmd_vel.angular.z = -MAX_ANGULAR_VEL

            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(LOOP_INTERVAL)

    # --- [회전 전용 콜백 라우팅] ---
    def execute_spin_rel_callback(self, goal_handle):
        self.get_logger().info(f'🔄 [상대 회전] {math.degrees(goal_handle.request.target_yaw):.1f}도 회전 요청됨')
        return self._perform_spin(goal_handle, is_absolute=False)

    def execute_spin_abs_callback(self, goal_handle):
        self.get_logger().info(f'🧭 [절대 회전] 맵 기준 {math.degrees(goal_handle.request.target_yaw):.1f}도로 회전 요청됨')
        return self._perform_spin(goal_handle, is_absolute=True)

    # --- [핵심 회전 제어 루프] ---
    def _perform_spin(self, goal_handle, is_absolute):
        while self.current_pose is None:
            time.sleep(0.1)

        start_yaw = self.get_yaw(self.current_pose.orientation)
        requested_yaw = goal_handle.request.target_yaw

        # 목표 각도(Goal Yaw) 계산
        if is_absolute:
            goal_yaw = self.normalize_angle(requested_yaw)
        else:
            goal_yaw = self.normalize_angle(start_yaw + requested_yaw)

        cmd_vel = Twist()
        feedback_msg = Spin.Feedback()
        result = Spin.Result()

        LOOP_INTERVAL = 0.05
        K_p = 2.0             # 회전 P게인 (전/후진보다 조금 더 세게 줘도 됩니다)
        ARRIVAL_ANGLE = 0.05  # 약 2.8도 이내면 도착으로 인정
        MIN_ANGULAR = 0.1     # 제자리에서 멈추지 않기 위한 최소 회전 속도
        MAX_ANGULAR = 0.8     # 최대 회전 속도

        # 누적 회전량 계산을 위한 변수
        last_yaw = start_yaw
        total_traveled = 0.0

        while True:
            # [A] 취소 요청
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('🛑 [Spin] 회전 취소됨')
                self.cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                return result

            # 1. 현재 각도 및 오차 계산
            curr_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(goal_yaw - curr_yaw)

            # 2. 누적 회전량(Feedback) 계산
            delta_yaw = abs(self.normalize_angle(curr_yaw - last_yaw))
            total_traveled += delta_yaw
            last_yaw = curr_yaw
            
            feedback_msg.angular_distance_traveled = total_traveled
            goal_handle.publish_feedback(feedback_msg)

            # [B] 도착 판별 (오차가 허용 범위 이내로 들어오면)
            if abs(yaw_error) <= ARRIVAL_ANGLE:
                self.get_logger().info(f'🏁 [Spin] 회전 완료! 남은 오차: {math.degrees(yaw_error):.1f}도')
                self.cmd_vel_pub.publish(Twist()) # 브레이크
                goal_handle.succeed()
                return result

            # 3. P-Control을 통한 회전 속도 계산 (자연스러운 감속 포함)
            angular_speed = K_p * yaw_error

            # 4. 속도 클램핑 (최댓값/최솟값 제한)
            # - 부호(방향)를 유지하면서 크기만 제한합니다.
            if angular_speed > 0:
                angular_speed = max(MIN_ANGULAR, min(MAX_ANGULAR, angular_speed))
            else:
                angular_speed = min(-MIN_ANGULAR, max(-MAX_ANGULAR, angular_speed))

            cmd_vel.angular.z = angular_speed
            self.cmd_vel_pub.publish(cmd_vel)
            
            time.sleep(LOOP_INTERVAL)


    def virtual_wall_callback(self, msg: Line2DFlt32Array):
        marker = Marker()
        marker.header.frame_id = 'slamware_map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'virtual_walls' # 네임스페이스 지정 (RViz에서 켜고 끄기 유용)
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # 선 굵기 및 색상 (RViz2 기준 0.1은 꽤 두꺼울 수 있으므로 0.05 추천)
        marker.scale.x = 0.05  
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        
        # 선분 데이터 추출 및 Point 변환
        for line in msg.lines:
            # line.start와 line.end가 각각 x, y 속성을 가지고 있다고 가정
            p_start = Point(x=float(line.start.x), y=float(line.start.y), z=0.0)
            p_end = Point(x=float(line.end.x), y=float(line.end.y), z=0.0)

            marker.points.append(p_start)
            marker.points.append(p_end)

        # 메인 도메인으로 변환된 Marker 퍼블리시
        self.virtual_wall_pub.publish(marker)


    def get_yaw(self, q):
        """쿼터니언에서 Yaw(z축 회전) 값을 추출합니다."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 사이로 정규화합니다."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    parser = argparse.ArgumentParser(description='ROS 2 Domain Bridge for Slamware')
    parser.add_argument('--main-domain', type=int, default=11)
    parser.add_argument('--slam-domain', type=int, default=35)
    args, _ = parser.parse_known_args()

    slam_context = Context()
    slam_context.init(domain_id=args.slam_domain)

    main_context = Context()
    main_context.init(domain_id=args.main_domain)

    bridge_node = None
    try:
        bridge_node = SlamwareBridge(slam_context, main_context)
        executor = rclpy.executors.MultiThreadedExecutor(context=main_context)
        executor.add_node(bridge_node)
        executor.add_node(bridge_node.slam_sub_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if bridge_node is not None:
            bridge_node.destroy_node()
            bridge_node.slam_sub_node.destroy_node()
        if slam_context.ok():
            slam_context.shutdown()
        if main_context.ok():
            main_context.shutdown()


if __name__ == '__main__':
    main()
