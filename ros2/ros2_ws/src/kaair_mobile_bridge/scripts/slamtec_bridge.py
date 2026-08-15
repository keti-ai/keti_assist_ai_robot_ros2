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
from slamware_ros_sdk.msg import (
    CancelActionRequest, Line2DFlt32Array, RotateRequest, RotateToRequest, RobotBasicState,
    MoveToRequest, MoveOptionFlag,
)
from kaair_msgs.srv import MobileShift, MobileRotate

from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import argparse
import math
import os
import time
import threading
from copy import deepcopy
from datetime import datetime
from zoneinfo import ZoneInfo


# rcutils의 {time} 토큰은 epoch 초 단위라 직관적이지 않고, 컨테이너의 시스템
# 타임존이 UTC로 설정된 경우가 많아 datetime.now()만 쓰면 한국 시간과 어긋나므로,
# Asia/Seoul 로 명시적으로 고정한 사람이 읽기 쉬운 시:분:초.밀리초 시간을 붙인다.
_KST = ZoneInfo("Asia/Seoul")


def _now() -> str:
    return datetime.now(_KST).strftime("%H:%M:%S.%f")[:-3]


class SlamwareBridge(Node):
    def __init__(self, slamware_context, main_context):
        super().__init__('mobile_bridge', context=main_context)
        self.cb_group = ReentrantCallbackGroup()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.slam_sub_node = Node('slamware_bridge_sub', context=slamware_context)
        self.current_pose = None
        self.active_goal_handle = None

        self.latest_battery_state = None
        self._log_throttle_last_time = {}
        self.latest_virtual_tracks = []  # [{'start': (x, y), 'end': (x, y)}, ...]

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
        self.declare_parameter('nav_stall_timeout_sec', 6.0)
        self.declare_parameter('nav_goal_attempt_count', 2)

        # NavigateToPose: virtual track(가상 트랙) 기반 이동 옵션
        # - 목적지(및 출발지)가 가상 트랙 근처에 있으면 move_to 토픽을 KEY_POINTS
        #   옵션으로 호출해 트랙을 따라 이동하도록 한다.
        self.declare_parameter('nav_virtual_track_proximity_m', 1.0)
        self.declare_parameter('nav_virtual_track_use_obstacle_avoidance', True)

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

        # NavigateToPose 액션 전용: virtual track을 활용할 수 있는 move_to 토픽
        self.move_to_pub = self.slam_sub_node.create_publisher(
            MoveToRequest, '/slamware_ros_sdk_server_node/move_to', 10)

        self.slam_cancel_pub = self.slam_sub_node.create_publisher(
            CancelActionRequest, '/slamware_ros_sdk_server_node/cancel_action', 10)

        # [메인 도메인] RViz2 시각화용 Marker 퍼블리셔 생성
        self.virtual_wall_pub = self.create_publisher(
            Marker, '/virtual_walls_marker', 10)  # 이름 충돌 방지를 위해 _marker 추가 권장
        self.virtual_track_pub = self.create_publisher(
            Marker, '/virtual_tracks_marker', 10)

        # [Slamware 도메인] 가상벽 데이터 구독
        self.virtual_wall_sub = self.slam_sub_node.create_subscription(
            Line2DFlt32Array, '/virtual_walls', 
            self.virtual_wall_callback, volatile_qos)

        # [Slamware 도메인] 가상 트랙(virtual track) 데이터 구독
        # move_to 시 목적지/출발지가 트랙 근처인지 판단하는 데 사용한다.
        self.virtual_track_sub = self.slam_sub_node.create_subscription(
            Line2DFlt32Array, '/slamware_ros_sdk_server_node/virtual_tracks',
            self.virtual_track_callback, volatile_qos)

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


        self._info(
            f'✅ Bridge Started: '
            f'[Main Domain: {main_context.get_domain_id()}] <-> '
            f'[Slamware Domain: {slamware_context.get_domain_id()}]'
        )

    def _should_log_throttled(self, key: str, throttle_duration_sec: float) -> bool:
        """자체 스로틀링 구현.

        rclpy의 get_logger().log(...)는 throttle_duration_sec 같은 필터 설정을
        '호출된 소스 라인'을 키로 캐싱하는데, _info/_warn/_err처럼 모든 로그가
        한 줄을 공유하는 래퍼를 거치면 서로 다른 호출부가 필터 유무/파라미터를
        다르게 넘기는 순간 "Requested logging filters cannot be changed between
        calls." 예외가 발생한다. 이를 피하기 위해 rclpy 내장 throttle 필터를
        쓰지 않고 여기서 직접 시간 간격을 체크한다.
        """
        now = time.monotonic()
        last = self._log_throttle_last_time.get(key, 0.0)
        if now - last < throttle_duration_sec:
            return False
        self._log_throttle_last_time[key] = now
        return True

    def _info(self, msg, *, throttle_duration_sec=None, throttle_key=None):
        if throttle_duration_sec is not None:
            if not self._should_log_throttled(throttle_key or msg, throttle_duration_sec):
                return
        self.get_logger().info(f'[{_now()}] {msg}')

    def _warn(self, msg, *, throttle_duration_sec=None, throttle_key=None):
        if throttle_duration_sec is not None:
            if not self._should_log_throttled(throttle_key or msg, throttle_duration_sec):
                return
        self.get_logger().warn(f'[{_now()}] {msg}')

    def _err(self, msg, *, throttle_duration_sec=None, throttle_key=None):
        if throttle_duration_sec is not None:
            if not self._should_log_throttled(throttle_key or msg, throttle_duration_sec):
                return
        self.get_logger().error(f'[{_now()}] {msg}')

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
        self._info(
            f'{BLUE}[NavigateToPose] ✅ Goal RECEIVED — '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}, '
            f'yaw={math.degrees(self.get_yaw(pose.orientation)):.1f}° '
            f'(frame: {goal_request.pose.header.frame_id}){RESET}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self._info('🛑 Received cancel request')
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
            
            self._info(
                f'{GREEN_COLOR}[Battery Status] Remaining: {pct}% | State: {status_str}{RESET_COLOR}'
            )
        else:
            # 아직 메시지를 수신하지 못한 경우 무겁지 않게 알림
            self._warn(
                '⏳ Waiting for Slamware battery state data...',
                throttle_duration_sec=5.0,
                throttle_key='battery_wait',
            )

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
        self._info(
            f'{BLUE}[NavigateToPose] 🏃 Execute START — '
            f'x={goal_pose_dbg.position.x:.3f}, y={goal_pose_dbg.position.y:.3f}, '
            f'yaw={math.degrees(self.get_yaw(goal_pose_dbg.orientation)):.1f}°{RESET}'
        )

        goal_pose = goal_handle.request.pose
        goal_pose.header.frame_id = 'slamware_map'

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
        VT_PROXIMITY_M         = self.get_parameter('nav_virtual_track_proximity_m').get_parameter_value().double_value
        VT_USE_OA              = self.get_parameter('nav_virtual_track_use_obstacle_avoidance').get_parameter_value().bool_value

        goal_yaw = self.get_yaw(goal_pose.pose.orientation)

        # 목적지(및 현재 위치)가 virtual track 근처인지 판단해 이동 모드를 결정한다.
        # 이 판단은 실행 시작 시 한 번만 하고, 재전송(재시도) 때도 동일 모드를 유지한다.
        check_points = [(goal_pose.pose.position.x, goal_pose.pose.position.y)]
        if self.current_pose is not None:
            check_points.append((self.current_pose.position.x, self.current_pose.position.y))
        use_virtual_track = self._is_virtual_track_nearby(check_points, VT_PROXIMITY_M)

        MAGENTA = "\033[95m"
        RESET_VT = "\033[0m"
        if use_virtual_track:
            self._info(
                f'{MAGENTA}[NavigateToPose] 🛤️ Virtual track 근접 감지(≤{VT_PROXIMITY_M:.1f}m) — '
                f'KEY_POINTS 모드로 move_to 이동{RESET_VT}'
            )
        else:
            self._info(
                f'{MAGENTA}[NavigateToPose] Virtual track 없음 — 일반 move_to 이동{RESET_VT}'
            )

        move_to_req = self._build_move_to_request(
            goal_pose.pose.position.x, goal_pose.pose.position.y, goal_yaw,
            use_virtual_track, VT_USE_OA
        )
        self.move_to_pub.publish(move_to_req)

        last_snapshot = None
        last_move_time = self.get_clock().now()
        start_time = self.get_clock().now()
        goal_attempt = 1

        while True:
            # [A-1] Preemption 체크
            if self.active_goal_handle != goal_handle:
                self._warn('🔄 [Preempt] 새 목표 수신. 기존 감시를 부드럽게 종료합니다.')
                goal_handle.canceled() 
                time.sleep(0.1) 
                return result

            # [A-2] 클라이언트 취소 요청 체크
            if goal_handle.is_cancel_requested:
                self._warn('🛑 [Cancel] 클라이언트에 의해 취소됨')
                from slamware_ros_sdk.msg import CancelActionRequest
                self.slam_cancel_pub.publish(CancelActionRequest())
                goal_handle.canceled()
                time.sleep(0.1) 
                return result

            now = self.get_clock().now()
            elapsed_total = (now - start_time).nanoseconds / 1e9

            # [B] 전체 타임아웃 체크
            if elapsed_total > TOTAL_TIMEOUT:
                self._err(f'⏳ [Timeout] 전체 제한시간 초과')
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
                        self._info('🔄 [Recovery] 로봇이 정지 상태를 탈출하여 다시 이동을 시작했습니다.')
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
                    self._info(
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
                        self._warn(
                            f'{YELLOW}[NavigateToPose] ⏳ 정지 타임아웃 ({STALL_TIMEOUT_FOR_REPLAN:.1f}s) — '
                            f'move_to 재전송 ({goal_attempt}/{MAX_GOAL_ATTEMPTS}), '
                            f'남은 거리: {dist_to_goal:.2f}m{RESET}'
                        )
                        self.move_to_pub.publish(move_to_req)
                        last_move_time = now
                        last_snapshot = None
                        continue

                    RED   = "\033[91m"
                    RESET = "\033[0m"
                    self._err(
                        f'{RED}[NavigateToPose] ⚠️ 막힘/고립 — '
                        f'재시도 {MAX_GOAL_ATTEMPTS}회 후 주행 실패: 남은 거리 {dist_to_goal:.2f}m{RESET}'
                    )
                    goal_handle.abort()
                    return result
                elif stall_duration > 2.0:
                    self._warn(
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
            self._warn('[Shift] distance=0, 이동 없음')
            response.successed = False
            return response

        duration = abs(distance / self.shift_velocity)
        direction = math.copysign(1.0, distance)
        CYAN  = "\033[96m"
        RESET = "\033[0m"
        direction_str = '전진' if direction > 0 else '후진'
        self._info(
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
        self._info(f'{GREEN}[Shift] ✅ 이동 완료{RESET}')

    # ------------------------------------------------------------------ #
    #  mobile/rotate  서비스 — 상대 각도 회전 (Slamware RotateRequest)
    # ------------------------------------------------------------------ #
    def service_callback_rotate(self, request: MobileRotate.Request, response: MobileRotate.Response):
        theta = request.theta
        if theta == 0.0:
            self._warn('[Rotate] theta=0, 회전 없음')
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
        self._info(f'{CYAN}[Rotate] 상대 회전 명령 — {math.degrees(theta):.1f}°{RESET}')

        if request.wait:
            time.sleep(1.0)
            while self.robot_is_moving:
                time.sleep(0.3)
            self._info(f'{GREEN}[Rotate] ✅ 회전 완료{RESET}')

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
        self._info(f'{CYAN}[AbsRotate] 절대 회전 명령 → {math.degrees(theta):.1f}°{RESET}')

        if request.wait:
            time.sleep(1.0)
            while self.robot_is_moving:
                time.sleep(0.3)
            self._info(f'{GREEN}[AbsRotate] ✅ 회전 완료{RESET}')

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

    def virtual_track_callback(self, msg: Line2DFlt32Array):
        """Slamware가 관리하는 virtual track 목록을 캐싱하고 RViz용 Marker로 발행한다.

        - 캐싱된 좌표는 NavigateToPose 실행 시 목적지가 트랙 근처인지 판단하는 데 사용된다.
        - virtual_walls와 동일하게 Line2DFlt32Array → Marker(LINE_LIST) 변환을 거쳐야
          RViz2에서 시각화할 수 있다 (Line2DFlt32Array 자체는 RViz가 지원하는 타입이 아님).
        """
        self.latest_virtual_tracks = [
            {
                'start': (float(line.start.x), float(line.start.y)),
                'end': (float(line.end.x), float(line.end.y)),
            }
            for line in msg.lines
        ]

        marker = Marker()
        marker.header.frame_id = 'slamware_map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'virtual_tracks'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # virtual_walls(빨강)와 구분되는 색상 (초록색)
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for track in self.latest_virtual_tracks:
            marker.points.append(Point(x=track['start'][0], y=track['start'][1], z=0.0))
            marker.points.append(Point(x=track['end'][0], y=track['end'][1], z=0.0))

        self.virtual_track_pub.publish(marker)

    @staticmethod
    def _point_to_segment_distance(px, py, ax, ay, bx, by):
        """점 (px,py)와 선분 (ax,ay)-(bx,by) 사이의 최단 거리."""
        abx = bx - ax
        aby = by - ay
        seg_len_sq = abx * abx + aby * aby
        if seg_len_sq < 1e-9:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * abx + (py - ay) * aby) / seg_len_sq
        t = max(0.0, min(1.0, t))
        closest_x = ax + t * abx
        closest_y = ay + t * aby
        return math.hypot(px - closest_x, py - closest_y)

    def _distance_to_nearest_virtual_track(self, x, y):
        """(x, y)에서 가장 가까운 virtual track까지의 거리. 트랙이 없으면 inf."""
        if not self.latest_virtual_tracks:
            return math.inf
        return min(
            self._point_to_segment_distance(
                x, y, track['start'][0], track['start'][1], track['end'][0], track['end'][1]
            )
            for track in self.latest_virtual_tracks
        )

    def _is_virtual_track_nearby(self, points, proximity_m):
        """points(여러 (x, y) 좌표) 중 하나라도 virtual track 근처면 True."""
        if not self.latest_virtual_tracks:
            return False
        return any(
            self._distance_to_nearest_virtual_track(x, y) <= proximity_m
            for (x, y) in points
        )

    def _build_move_to_request(self, x, y, yaw, use_virtual_track, use_obstacle_avoidance):
        """MoveToRequest 메시지 생성.

        - virtual track 모드: KEY_POINTS (+ KEY_POINTS_WITH_OA) 플래그로
          트랙을 따라 이동. 로봇이 트랙 경로를 우선 사용한다.
        - 일반 모드: 기존 /move_base_simple/goal 핸들러와 동일하게
          MILESTONE + PRECISE 플래그 사용.
        - 두 모드 모두 WITH_YAW로 최종 목적지의 yaw를 지정한다.
        """
        req = MoveToRequest()
        req.location.x = float(x)
        req.location.y = float(y)
        req.location.z = 0.0
        req.yaw = float(yaw)

        flags = MoveOptionFlag.WITH_YAW | MoveOptionFlag.PRECISE
        if use_virtual_track:
            flags |= MoveOptionFlag.KEY_POINTS
            if use_obstacle_avoidance:
                flags |= MoveOptionFlag.KEY_POINTS_WITH_OA
        else:
            flags |= MoveOptionFlag.MILESTONE
        req.options.opt_flags.flags = flags

        return req

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
    # 기본 {time} 토큰(epoch 초)이 안 보이도록 콘솔 출력 포맷을 정리한다.
    # 사용자가 이미 RCUTILS_CONSOLE_OUTPUT_FORMAT 을 설정했다면 그대로 존중한다.
    os.environ.setdefault("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}")

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

