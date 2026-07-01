#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Point
from visualization_msgs.msg import Marker
from slamware_ros_sdk.msg import CancelActionRequest, Line2DFlt32Array, RotateRequest, RotateToRequest, RobotBasicState
from kaair_msgs.srv import MobileShift, MobileRotate

from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import argparse
import math
import time
import threading
from copy import deepcopy


class SlamwareBridge(Node):
    def __init__(self, slamware_context, main_context):
        super().__init__('mobile_bridge', context=main_context)
        self.cb_group = ReentrantCallbackGroup()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.slam_sub_node = Node('slamware_bridge_sub', context=slamware_context)
        self.current_pose = None
        self.active_goal_handle = None

        self.latest_battery_state = None

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # 서비스 서버 (main domain에서 직접 호출 가능)
        self.srv_shift = self.create_service(
            MobileShift, 'mobile/shift_pose',
            self.service_callback_shift_pose,
            callback_group=self.cb_group
        )
        self.srv_rotate = self.create_service(
            MobileRotate, 'mobile/rotate',
            self.service_callback_rotate,
            callback_group=self.cb_group
        )
        self.srv_abs_rotate = self.create_service(
            MobileRotate, 'mobile/absolute_rotate',
            self.service_callback_absolute_rotate,
            callback_group=self.cb_group
        )

        # 로봇 이동 상태 추적
        self.robot_is_moving = False
        self.last_robot_pose = None
        self.consecutive_no_change_counts = 0
        self.REQUIRED_NO_CHANGE_COUNTS = 10
        self.movement_threshold_linear = 0.005   # m
        self.movement_threshold_angular = math.radians(0.5)
        self.shift_velocity = 0.15  # m/s

        # NavigateToPose: 정지 감지 타임아웃 및 goal 재전송 횟수
        self.declare_parameter('nav_stall_timeout_sec', 5.0)
        self.declare_parameter('nav_goal_attempt_count', 2)
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

        self.rotate_pub = self.slam_sub_node.create_publisher(
            RotateRequest, '/slamware_ros_sdk_server_node/rotate', 10)
        self.rotate_to_pub = self.slam_sub_node.create_publisher(
            RotateToRequest, '/slamware_ros_sdk_server_node/rotate_to', 10)

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

        # [Slamware 도메인] 로봇 기본 상태(배터리 포함) 토픽 구독 추가
        self.basic_state_sub = self.slam_sub_node.create_subscription(
            RobotBasicState, 
            '/slamware_ros_sdk_server_node/robot_basic_state', 
            self.basic_state_callback, 
            volatile_qos
        )

        # [메인 도메인] 1.5초 간격으로 배터리 상태를 로깅하는 주기적 타이머 생성
        self.battery_log_timer = self.create_timer(
            5.0, 
            self.battery_log_timer_callback, 
            callback_group=self.cb_group
        )


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

        # 이동 감지 (서비스 wait 기능에 사용)
        if self.last_robot_pose is not None:
            dx = msg.pose.position.x - self.last_robot_pose.pose.position.x
            dy = msg.pose.position.y - self.last_robot_pose.pose.position.y
            linear_dist = math.sqrt(dx**2 + dy**2)
            curr_yaw = self.get_yaw(msg.pose.orientation)
            last_yaw = self.get_yaw(self.last_robot_pose.pose.orientation)
            angular_diff = abs(self.normalize_angle(curr_yaw - last_yaw))

            if linear_dist > self.movement_threshold_linear or angular_diff > self.movement_threshold_angular:
                self.robot_is_moving = True
                self.consecutive_no_change_counts = 0
            else:
                self.consecutive_no_change_counts += 1
                if self.consecutive_no_change_counts >= self.REQUIRED_NO_CHANGE_COUNTS:
                    self.robot_is_moving = False
        self.last_robot_pose = msg

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
        BLUE  = "\033[94m"
        RESET = "\033[0m"
        pose  = goal_request.pose.pose
        self.get_logger().info(
            f'{BLUE}[NavigateToPose] ✅ Goal RECEIVED — '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}, '
            f'yaw={math.degrees(self.get_yaw(pose.orientation)):.1f}° '
            f'(frame: {goal_request.pose.header.frame_id}){RESET}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('🛑 Received cancel request')
        return CancelResponse.ACCEPT

    def basic_state_callback(self, msg):
        self.latest_battery_state = msg

    # 로그 출력을 위한 타이머 콜백 함수
    def battery_log_timer_callback(self):
        if self.latest_battery_state is not None:
            pct = self.latest_battery_state.battery_percentage
            charging = self.latest_battery_state.is_charging
            dc_in = self.latest_battery_state.is_dc_in
            
            # 충전 상태 스트링 조절
            status_str = "Charging" if charging else ("DC-IN Connected" if dc_in else "Discharging")
            
            # ANSI 이스케이프 코드: \033[92m (밝은 초록색) / \033[0m (원래대로 색상 리셋)
            GREEN_COLOR = "\033[92m"
            RESET_COLOR = "\033[0m"
            
            self.get_logger().info(
                f'{GREEN_COLOR}[Battery Status] Remaining: {pct}% | State: {status_str}{RESET_COLOR}'
            )
        else:
            # 아직 메시지를 수신하지 못한 경우 무겁지 않게 알림
            self.get_logger().warn('⏳ Waiting for Slamware battery state data...', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ #
    #  실제 네비게이션 로직 - 별도 스레드에서 동기 실행
    # ------------------------------------------------------------------ #
# ------------------------------------------------------------------ #
    #  execute_callback - 일반 동기 함수 (MultiThreadedExecutor가 스레드 자동 할당)
    # ------------------------------------------------------------------ #
    # def execute_callback(self, goal_handle):
    #     # 🌟 새 목표가 들어오면 현재 활성 목표를 이 goal_handle로 덮어씁니다.
    #     self.active_goal_handle = goal_handle

    #     self.get_logger().info('🏃 [1] 액션 콜백 진입: 목표 퍼블리시 시작...')

    #     goal_pose = goal_handle.request.pose
    #     goal_pose.header.frame_id = 'slamware_map'
    #     self.goal_pub.publish(goal_pose)

    #     feedback_msg = NavigateToPose.Feedback()
    #     result = NavigateToPose.Result()

    #     STALL_POS_THRESHOLD = 0.02   
    #     STALL_YAW_THRESHOLD = 0.05   
    #     STALL_TIME_LIMIT    = 3.0    
    #     TOTAL_TIMEOUT       = 180.0  
    #     ARRIVAL_DISTANCE    = 0.25   
    #     ARRIVAL_ANGLE       = 0.15   
    #     LOOP_INTERVAL       = 0.5

    #     goal_yaw = self.get_yaw(goal_pose.pose.orientation)

    #     last_snapshot = None
    #     last_move_time = self.get_clock().now()
    #     start_time = self.get_clock().now()

    #     while True:
    #         # 🌟 [A-1] 탈출구 0: 새로운 목표가 들어온 경우 (Preemption)
    #         if self.active_goal_handle != goal_handle:
    #             self.get_logger().warn('🔄 [Preempt] 새 목표 수신. 기존 감시를 부드럽게 종료합니다.')
    #             # RViz2 튕김 방지 1: abort() 대신 canceled() 사용
    #             goal_handle.canceled() 
    #             # RViz2 튕김 방지 2: C++ 클라이언트가 상태를 받을 수 있도록 아주 잠깐 대기
    #             time.sleep(0.1) 
    #             return result

    #         # 🌟 [A-2] 탈출구 1: 클라이언트 취소 요청 (Cancel 버튼)
    #         if goal_handle.is_cancel_requested:
    #             self.get_logger().warn('🛑 [Cancel] 클라이언트에 의해 취소됨')
                
    #             from slamware_ros_sdk.msg import CancelActionRequest
    #             self.slam_cancel_pub.publish(CancelActionRequest())
                
    #             goal_handle.canceled()
    #             # RViz2 튕김 방지 2: 취소 상태가 DDS 통신망을 타고 RViz2에 도착할 시간을 줍니다.
    #             time.sleep(0.1) 
    #             return result

    #         now = self.get_clock().now()
    #         elapsed_total = (now - start_time).nanoseconds / 1e9

    #         # [B] 탈출구 2: 타임아웃
    #         if elapsed_total > TOTAL_TIMEOUT:
    #             self.get_logger().error(f'⏳ [Timeout] 전체 제한시간 초과')
    #             goal_handle.abort()
    #             return result

    #         if self.current_pose is None:
    #             time.sleep(LOOP_INTERVAL)
    #             continue

    #         current_snapshot = deepcopy(self.current_pose)
    #         current_yaw = self.get_yaw(current_snapshot.orientation)

    #         # --- 위치 및 각도 오차 계산 ---
    #         dx = goal_pose.pose.position.x - current_snapshot.position.x
    #         dy = goal_pose.pose.position.y - current_snapshot.position.y
    #         dist_to_goal = math.sqrt(dx**2 + dy**2)
            
    #         # 🌟 [180도 회전 허용 로직]
    #         raw_yaw_error = abs(self.normalize_angle(goal_yaw - current_yaw))
            
    #         # 오차가 90도(pi/2) 이상 나면, 반대쪽(180도 뒤집힌 상태)으로 계산
    #         # 즉, 로봇이 앞을 보든 뒤를 보든 일직선상으로 정렬되면 오차를 0에 가깝게 만듭니다.
    #         if raw_yaw_error > (math.pi / 2.0):
    #             yaw_error = abs(math.pi - raw_yaw_error)
    #         else:
    #             yaw_error = raw_yaw_error

    #         feedback_msg.distance_remaining = dist_to_goal
    #         goal_handle.publish_feedback(feedback_msg)

    #         self.get_logger().info(
    #             f'📍 거리: {dist_to_goal:.2f}m | 각도 오차: {math.degrees(yaw_error):.1f}° | '
    #             f'정지 시간: {(now - last_move_time).nanoseconds / 1e9:.1f}s',
    #             throttle_duration_sec=1.0
    #         )

    #         # [D] 정지(Stall) 체크 (위치 & 회전)
    #         if last_snapshot is not None:
    #             move_delta = math.sqrt(
    #                 (current_snapshot.position.x - last_snapshot.position.x)**2 +
    #                 (current_snapshot.position.y - last_snapshot.position.y)**2
    #             )
    #             last_yaw = self.get_yaw(last_snapshot.orientation)
    #             yaw_delta = abs(self.normalize_angle(current_yaw - last_yaw))

    #             if move_delta > STALL_POS_THRESHOLD or yaw_delta > STALL_YAW_THRESHOLD:
    #                 last_move_time = now

    #         last_snapshot = current_snapshot

    #         # [E] 정지 상태 초과 시 최종 판별
    #         stall_duration = (now - last_move_time).nanoseconds / 1e9
    #         if stall_duration > STALL_TIME_LIMIT:
    #             if dist_to_goal < ARRIVAL_DISTANCE and yaw_error < ARRIVAL_ANGLE:
    #                 self.get_logger().info(f'🏁 [도착 성공] 오차: {dist_to_goal:.2f}m, {math.degrees(yaw_error):.1f}°')
    #                 goal_handle.succeed()
    #             else:
    #                 self.get_logger().error(f'⚠️ [막힘/고립] 남은 거리: {dist_to_goal:.2f}m, 각도 오차: {math.degrees(yaw_error):.1f}°')
    #                 goal_handle.abort()
    #             return result

    #         time.sleep(LOOP_INTERVAL)

    def execute_callback(self, goal_handle):
        self.active_goal_handle = goal_handle
        BLUE  = "\033[94m"
        RESET = "\033[0m"
        goal_pose_dbg = goal_handle.request.pose.pose
        self.get_logger().info(
            f'{BLUE}[NavigateToPose] 🏃 Execute START — '
            f'x={goal_pose_dbg.position.x:.3f}, y={goal_pose_dbg.position.y:.3f}, '
            f'yaw={math.degrees(self.get_yaw(goal_pose_dbg.orientation)):.1f}°{RESET}'
        )

        goal_pose = goal_handle.request.pose
        goal_pose.header.frame_id = 'slamware_map'
        self.goal_pub.publish(goal_pose)

        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # --- 파라미터 튜닝 조절 ---
        STALL_POS_THRESHOLD    = 0.02   
        STALL_YAW_THRESHOLD    = 0.05   
        ARRIVAL_DISTANCE       = 0.25   
        ARRIVAL_ANGLE          = 0.15   
        TOTAL_TIMEOUT          = 180.0  
        LOOP_INTERVAL          = 0.5

        STALL_TIME_NEAR_GOAL   = 3.0    # 목적지 근처에서 꼈을 때의 마진 (초)
        STALL_TIMEOUT_FOR_REPLAN = self.get_parameter('nav_stall_timeout_sec').get_parameter_value().double_value
        MAX_GOAL_ATTEMPTS      = max(1, self.get_parameter('nav_goal_attempt_count').get_parameter_value().integer_value)

        goal_yaw = self.get_yaw(goal_pose.pose.orientation)

        last_snapshot = None
        last_move_time = self.get_clock().now()
        start_time = self.get_clock().now()
        goal_attempt = 1

        while True:
            # [A-1] Preemption 체크
            if self.active_goal_handle != goal_handle:
                self.get_logger().warn('🔄 [Preempt] 새 목표 수신. 기존 감시를 부드럽게 종료합니다.')
                goal_handle.canceled() 
                time.sleep(0.1) 
                return result

            # [A-2] 클라이언트 취소 요청 체크
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('🛑 [Cancel] 클라이언트에 의해 취소됨')
                from slamware_ros_sdk.msg import CancelActionRequest
                self.slam_cancel_pub.publish(CancelActionRequest())
                goal_handle.canceled()
                time.sleep(0.1) 
                return result

            now = self.get_clock().now()
            elapsed_total = (now - start_time).nanoseconds / 1e9

            # [B] 전체 타임아웃 체크
            if elapsed_total > TOTAL_TIMEOUT:
                self.get_logger().error(f'⏳ [Timeout] 전체 제한시간 초과')
                goal_handle.abort()
                return result

            if self.current_pose is None:
                time.sleep(LOOP_INTERVAL)
                continue

            current_snapshot = deepcopy(self.current_pose)
            current_yaw = self.get_yaw(current_snapshot.orientation)

            # 위치 및 각도 오차 계산
            dx = goal_pose.pose.position.x - current_snapshot.position.x
            dy = goal_pose.pose.position.y - current_snapshot.position.y
            dist_to_goal = math.sqrt(dx**2 + dy**2)
            
            raw_yaw_error = abs(self.normalize_angle(goal_yaw - current_yaw))
            if raw_yaw_error > (math.pi / 2.0):
                yaw_error = abs(math.pi - raw_yaw_error)
            else:
                yaw_error = raw_yaw_error

            feedback_msg.distance_remaining = dist_to_goal
            goal_handle.publish_feedback(feedback_msg)

            # --- [D] 정지(Stall) 및 복귀 체크 ---
            if last_snapshot is not None:
                move_delta = math.sqrt(
                    (current_snapshot.position.x - last_snapshot.position.x)**2 +
                    (current_snapshot.position.y - last_snapshot.position.y)**2
                )
                last_yaw = self.get_yaw(last_snapshot.orientation)
                yaw_delta = abs(self.normalize_angle(current_yaw - last_yaw))

                # 🌟 조금이라도 움직임이 감지되면 '마지막 이동 시간'을 갱신 (Timer Reset)
                if move_delta > STALL_POS_THRESHOLD or yaw_delta > STALL_YAW_THRESHOLD:
                    if (now - last_move_time).nanoseconds / 1e9 > 2.0:
                        self.get_logger().info('🔄 [Recovery] 로봇이 정지 상태를 탈출하여 다시 이동을 시작했습니다.')
                    last_move_time = now

            last_snapshot = current_snapshot

            # --- [E] 🌟 조건별 유연한 정지(Stall) 판별 루틴 ---
            stall_duration = (now - last_move_time).nanoseconds / 1e9
            
            # 목적지 판정 영역(Safe Zone) 내부인지 외부에 있는지에 따라 타임아웃 마진을 다르게 적용
            if dist_to_goal < ARRIVAL_DISTANCE and yaw_error < ARRIVAL_ANGLE:
                # 목적지 근처인 경우: 기존처럼 짧은 타임아웃(3초) 적용 후 성공 처리 판별
                if stall_duration > STALL_TIME_NEAR_GOAL:
                    GREEN = "\033[92m"
                    RESET = "\033[0m"
                    self.get_logger().info(
                        f'{GREEN}[NavigateToPose] 🏁 도착 성공 — '
                        f'오차: {dist_to_goal:.2f}m, {math.degrees(yaw_error):.1f}°{RESET}'
                    )
                    goal_handle.succeed()
                    return result
            else:
                if stall_duration > STALL_TIMEOUT_FOR_REPLAN:
                    if goal_attempt < MAX_GOAL_ATTEMPTS:
                        goal_attempt += 1
                        YELLOW = "\033[93m"
                        RESET  = "\033[0m"
                        self.get_logger().warn(
                            f'{YELLOW}[NavigateToPose] ⏳ 정지 타임아웃 ({STALL_TIMEOUT_FOR_REPLAN:.1f}s) — '
                            f'move topic 재전송 ({goal_attempt}/{MAX_GOAL_ATTEMPTS}), '
                            f'남은 거리: {dist_to_goal:.2f}m{RESET}'
                        )
                        goal_pose.header.stamp = self.get_clock().now().to_msg()
                        self.goal_pub.publish(goal_pose)
                        last_move_time = now
                        last_snapshot = None
                        continue

                    RED   = "\033[91m"
                    RESET = "\033[0m"
                    self.get_logger().error(
                        f'{RED}[NavigateToPose] ⚠️ 막힘/고립 — '
                        f'재시도 {MAX_GOAL_ATTEMPTS}회 후 주행 실패: 남은 거리 {dist_to_goal:.2f}m{RESET}'
                    )
                    goal_handle.abort()
                    return result
                elif stall_duration > 2.0:
                    self.get_logger().warn(
                        f'⏳ [Stagnation Detection] 로봇 일시 정지 중 (남은 거리: {dist_to_goal:.2f}m). '
                        f'정지 타임아웃: {stall_duration:.1f}s / {STALL_TIMEOUT_FOR_REPLAN}s '
                        f'(시도 {goal_attempt}/{MAX_GOAL_ATTEMPTS})',
                        throttle_duration_sec=2.0
                    )

            time.sleep(LOOP_INTERVAL)


    # ------------------------------------------------------------------ #
    #  mobile/shift_pose  서비스 — 전진/후진 (시간 기반 cmd_vel)
    # ------------------------------------------------------------------ #
    def service_callback_shift_pose(self, request: MobileShift.Request, response: MobileShift.Response):
        distance = request.distance
        if distance == 0.0:
            self.get_logger().warn('[Shift] distance=0, 이동 없음')
            response.successed = False
            return response

        duration = abs(distance / self.shift_velocity)
        direction = math.copysign(1.0, distance)
        CYAN  = "\033[96m"
        RESET = "\033[0m"
        direction_str = '전진' if direction > 0 else '후진'
        self.get_logger().info(
            f'{CYAN}[Shift] {direction_str} 명령 — {abs(distance):.2f}m ({duration:.2f}s){RESET}'
        )

        if request.wait:
            self._move_linear(direction, duration)
        else:
            threading.Thread(target=self._move_linear, args=(direction, duration), daemon=True).start()

        response.successed = True
        return response

    def _move_linear(self, direction: float, duration: float):
        twist = Twist()
        twist.linear.x = direction * self.shift_velocity
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        GREEN = "\033[92m"
        RESET = "\033[0m"
        self.get_logger().info(f'{GREEN}[Shift] ✅ 이동 완료{RESET}')

    # ------------------------------------------------------------------ #
    #  mobile/rotate  서비스 — 상대 각도 회전 (Slamware RotateRequest)
    # ------------------------------------------------------------------ #
    def service_callback_rotate(self, request: MobileRotate.Request, response: MobileRotate.Response):
        theta = request.theta
        if theta == 0.0:
            self.get_logger().warn('[Rotate] theta=0, 회전 없음')
            response.successed = False
            return response

        rotate_msg = RotateRequest()
        rotate_msg.rotation.x = 0.0
        rotate_msg.rotation.y = 0.0
        rotate_msg.rotation.z = math.sin(theta / 2.0)
        rotate_msg.rotation.w = math.cos(theta / 2.0)
        CYAN  = "\033[96m"
        GREEN = "\033[92m"
        RESET = "\033[0m"
        self.rotate_pub.publish(rotate_msg)
        self.get_logger().info(f'{CYAN}[Rotate] 상대 회전 명령 — {math.degrees(theta):.1f}°{RESET}')

        if request.wait:
            time.sleep(1.0)
            while self.robot_is_moving:
                time.sleep(0.3)
            self.get_logger().info(f'{GREEN}[Rotate] ✅ 회전 완료{RESET}')

        response.successed = True
        return response

    # ------------------------------------------------------------------ #
    #  mobile/absolute_rotate  서비스 — 절대 각도 회전 (Slamware RotateToRequest)
    # ------------------------------------------------------------------ #
    def service_callback_absolute_rotate(self, request: MobileRotate.Request, response: MobileRotate.Response):
        theta = request.theta
        rotate_msg = RotateToRequest()
        rotate_msg.orientation.x = 0.0
        rotate_msg.orientation.y = 0.0
        rotate_msg.orientation.z = math.sin(theta / 2.0)
        rotate_msg.orientation.w = math.cos(theta / 2.0)
        CYAN  = "\033[96m"
        GREEN = "\033[92m"
        RESET = "\033[0m"
        self.rotate_to_pub.publish(rotate_msg)
        self.get_logger().info(f'{CYAN}[AbsRotate] 절대 회전 명령 → {math.degrees(theta):.1f}°{RESET}')

        if request.wait:
            time.sleep(1.0)
            while self.robot_is_moving:
                time.sleep(0.3)
            self.get_logger().info(f'{GREEN}[AbsRotate] ✅ 회전 완료{RESET}')

        response.successed = True
        return response


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
    parser.add_argument('--main-domain', type=int, default=9)
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
