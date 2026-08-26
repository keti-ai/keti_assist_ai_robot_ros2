#!/usr/bin/env python3
"""호스트(메인) 도메인 <-> 로봇(Clobot) PC 간 브리지 (rosbridge/websocket 버전).

이전 버전은 rclpy Context 2개(도메인 10/35)를 한 프로세스에서 직접 이어붙이는
방식이었는데, 도메인 35 쪽 DDS 트래픽(특히 /map 같은 큰 메시지의 TRANSIENT_LOCAL
재전송)이 제한된 네트워크(FastRTPS interfaceWhiteList)에서 불안정했다.

이 버전은 DDS 도메인 브리징을 아예 쓰지 않는다. 로봇 PC에서 rosbridge_server를
띄워두고, 이 노드는 roslibpy로 그 websocket에 접속해 필요한 토픽/액션만
JSON으로 가져온다. 그래서 이 노드는 도메인이 하나뿐인 평범한 rclpy 노드다
(ROS_DOMAIN_ID 그대로 사용, FASTRTPS_DEFAULT_PROFILES_FILE 같은 특수 설정 불필요).

사전 준비:
    1) 로봇 PC에 rosbridge_suite 설치 후 실행 (예: humble)
        sudo apt install ros-humble-rosbridge-suite
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
    2) 이 컨테이너 쪽은 requirements.txt에 추가된 roslibpy가 필요 (docker 이미지
       재빌드 시 자동 설치됨).

⚠️ 액션 취소: roslibpy.ActionClient.cancel_goal()은 메시지를 보내긴 하지만,
rosbridge_suite 쪽 async cancel 지원이 버전에 따라 없을 수 있다(rosbridge_suite
issue #909). 취소가 실제로 로봇에 전달되는지는 실기 확인이 필요하다.

⚠️ TF 브리지: Clobot의 실제 TF 트리는 map -> odom -> base_footprint 2홉 구조로
확인됐다(map -> base_footprint 직접 연결이 아님). tf2_ros.Buffer 없이 순수 JSON
메시지만으로 이 두 변환을 직접 합성(quaternion 곱)해서 map -> base_footprint를
계산한다. 그 이상의 임의 깊이 트리는 지원하지 않는다.
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped, Twist
from kaair_msgs.srv import MobileRotate, MobileShift
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

import roslibpy
from roslibpy import ActionClient, Goal, GoalStatus

HOST_FRAME_ID = 'slamware_map'   # 호스트(메인) 도메인에서 사용하는 프레임 이름
ROBOT_FRAME_ID = 'map'           # 로봇(Clobot) 쪽 navigate_to_pose/TF의 map 프레임
ODOM_FRAME_ID = 'odom'           # map과 base_footprint 사이의 중간 프레임 (실측 확인됨)
BASE_FRAME_ID = 'base_footprint'  # 로봇 베이스 프레임 (양쪽 동일)
MAP_REPUBLISH_PERIOD_SEC = 2.0    # 캐싱된 /map을 호스트 도메인에 재발행하는 주기
GOAL_TIMEOUT_SEC = 180.0          # navigate_to_pose 목표 완료 대기 타임아웃
BATTERY_LOG_PERIOD_SEC = 5.0      # 배터리 상태를 터미널에 로깅하는 주기
MAX_BATTERY_VOLTAGE = 29.2        # 완충 추정 전압 (7cell, 충전기 output 28.8V 기준 실측 추정치)

# move_cmd_node/action/MoveCmd — 로봇 PC 자체 액션(전후진/상대회전만 지원).
# 절대각도 회전(mobile/absolute_rotate)은 TF에서 얻은 현재 yaw와 목표각의 차이를
# 최소 회전 방향으로 정규화해 상대회전으로 변환하는 방식으로 브리지에서 구현한다.
MOVE_CMD_ACTION_TYPE = 'move_cmd_node/action/MoveCmd'
MOVE_CMD_SHIFT = 1    # command_type: 전후진 (goal=거리[m], 부호로 방향)
MOVE_CMD_ROTATE = 2   # command_type: 상대 회전 (goal=각도[rad])
MOVE_CMD_LINEAR_SPEED = 0.7   # m/s
MOVE_CMD_ANGULAR_SPEED = 1.0  # rad/s
MOVE_CMD_MIN_TIMEOUT_SEC = 5.0    # 로봇에 넘기는 timeout_time의 하한
MOVE_CMD_WAIT_MARGIN_SEC = 3.0    # 서비스 쪽 대기 시간 = 로봇 timeout_time + 여유
ABS_ROTATE_EPSILON_RAD = 0.01     # 이 이하 차이는 "이미 목표 각도"로 보고 회전 생략

# 입력 수신/완료 로그를 한눈에 구분하기 위한 ANSI 색상
LOG_BLUE = '\033[94m'   # 요청/goal 수신
LOG_GREEN = '\033[92m'  # 완료(성공)
LOG_RED = '\033[91m'    # 완료(실패)
LOG_RESET = '\033[0m'


class ClobotWebsocketBridge(Node):
    """rosbridge websocket(roslibpy)으로 로봇 PC와 통신하며, 호스트 도메인에는
    평범한 rclpy 토픽/액션으로 노출한다.
    """

    def __init__(self, ros, robot_host, robot_port):
        super().__init__('clobot_bridge')
        self.cb_group = ReentrantCallbackGroup()
        self.ros = ros

        # --- 로봇 현재 위치(TF) 브리지: map -> odom -> base_footprint 합성 ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_pose = None
        self._tf_cache = {}  # (frame_id, child_frame_id) -> {'translation': {...}, 'rotation': {...}}
        self._tf_topic = roslibpy.Topic(self.ros, '/tf', 'tf2_msgs/TFMessage')
        self._tf_topic.subscribe(self._on_tf)
        self._tf_static_topic = roslibpy.Topic(self.ros, '/tf_static', 'tf2_msgs/TFMessage')
        self._tf_static_topic.subscribe(self._on_tf)

        # --- 맵(OccupancyGrid) 브리지: 캐싱 + 타이머 재발행 ---
        # 원본 /map과 동일하게 TRANSIENT_LOCAL로 발행해 늦게 붙는 구독자(RViz2 등)도
        # 다음 타이머 주기를 기다리지 않고 마지막 맵을 즉시 받게 한다.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._latest_map = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self._map_topic = roslibpy.Topic(self.ros, '/map', 'nav_msgs/OccupancyGrid')
        self._map_topic.subscribe(self._on_map)
        self._map_republish_timer = self.create_timer(
            MAP_REPUBLISH_PERIOD_SEC, self._publish_cached_map)

        # --- 라이다(/scan) 브리지: 수신 즉시 그대로 통과 ---
        # frame_id는 URDF(robot_state_publisher)가 이미 라이다 프레임을
        # 정의하고 있으므로 변환 없이 원본 값을 그대로 쓴다.
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self._scan_topic = roslibpy.Topic(self.ros, '/scan', 'sensor_msgs/LaserScan')
        self._scan_topic.subscribe(self._on_scan)

        # --- 배터리 상태 로깅 ---
        self._latest_battery = None
        self._battery_topic = roslibpy.Topic(
            self.ros, '/former_io_controller/battery_state', 'sensor_msgs/BatteryState')
        self._battery_topic.subscribe(self._on_battery)
        self._battery_log_timer = self.create_timer(
            BATTERY_LOG_PERIOD_SEC, self._log_battery_status)

        # --- 초기 위치(/initialpose) 브리지: 호스트 -> 로봇 (역방향) ---
        # RViz2의 "2D Pose Estimate"가 호스트 도메인에 slamware_map 프레임으로
        # 발행하는 걸 로봇의 map 프레임으로 바꿔 로봇 쪽 /initialpose로 전달한다.
        # roslibpy.Topic.publish()는 필요 시 알아서 advertise하므로 별도 호출 불필요.
        self._initialpose_ws_topic = roslibpy.Topic(
            self.ros, '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped')
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)

        # --- cmd_vel 브리지: 호스트 -> 로봇 (역방향) ---
        # 호스트 도메인의 /cmd_vel을 로봇의 /base_controller/cmd_vel_unstamped로 전달한다.
        self._cmd_vel_ws_topic = roslibpy.Topic(
            self.ros, '/base_controller/cmd_vel_unstamped', 'geometry_msgs/Twist')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # --- mobile/shift_pose, mobile/rotate 서비스 ---
        # 로봇 PC 자체 액션(move_cmd_node/move_cmd_action)에 goal을 보내고,
        # 그 액션이 끝날 때까지 기다렸다가 서비스 response를 돌려준다.
        self._move_cmd_client = ActionClient(
            self.ros, '/move_cmd_node/move_cmd_action', MOVE_CMD_ACTION_TYPE)
        self.srv_shift = self.create_service(
            MobileShift, 'mobile/shift_pose',
            self.service_callback_shift_pose,
            callback_group=self.cb_group,
        )
        self.srv_rotate = self.create_service(
            MobileRotate, 'mobile/rotate',
            self.service_callback_rotate,
            callback_group=self.cb_group,
        )
        # move_cmd_node는 상대회전만 지원해서, 절대각도는 현재 TF(slamware_map
        # 기준 yaw)를 읽어 목표각과의 최소 회전 방향 차이를 구한 뒤 상대회전으로
        # 변환해서 보낸다.
        self.srv_absolute_rotate = self.create_service(
            MobileRotate, 'mobile/absolute_rotate',
            self.service_callback_absolute_rotate,
            callback_group=self.cb_group,
        )

        # --- navigate_to_pose 액션 브리지 ---
        self._action_client = ActionClient(
            self.ros, '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose')

        # cancel_callback이 execute_callback 실행 중에도 즉시 처리될 수 있도록
        # ReentrantCallbackGroup + MultiThreadedExecutor 조합이 필요하다(기본
        # SingleThreadedExecutor면 execute_callback의 폴링 루프가 끝날 때까지
        # cancel_callback 자체가 호출되지 않는다).
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f'clobot_bridge(websocket) started -> ws://{robot_host}:{robot_port}')

    # ------------------------------------------------------------------ #
    #  TF
    # ------------------------------------------------------------------ #
    def _on_tf(self, msg):
        for t in msg.get('transforms', []):
            header = t.get('header', {})
            key = (header.get('frame_id'), t.get('child_frame_id'))
            self._tf_cache[key] = t.get('transform', {})
        self._publish_base_pose()

    def _publish_base_pose(self):
        direct = self._tf_cache.get((ROBOT_FRAME_ID, BASE_FRAME_ID))
        if direct is not None:
            translation, rotation = self._split_transform(direct)
        else:
            map_to_odom = self._tf_cache.get((ROBOT_FRAME_ID, ODOM_FRAME_ID))
            odom_to_base = self._tf_cache.get((ODOM_FRAME_ID, BASE_FRAME_ID))
            if map_to_odom is None or odom_to_base is None:
                return
            t1, q1 = self._split_transform(map_to_odom)
            t2, q2 = self._split_transform(odom_to_base)
            rotation = self._quat_mul(q1, q2)
            rotated_t2 = self._rotate_vec(q1, t2)
            translation = tuple(a + b for a, b in zip(t1, rotated_t2))

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
        self.current_pose = pose

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = HOST_FRAME_ID
        out.child_frame_id = BASE_FRAME_ID
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = translation
        out.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(out)

    @staticmethod
    def _split_transform(t):
        translation = t.get('translation', {})
        rotation = t.get('rotation', {})
        return (
            (float(translation.get('x', 0.0)), float(translation.get('y', 0.0)), float(translation.get('z', 0.0))),
            (float(rotation.get('x', 0.0)), float(rotation.get('y', 0.0)),
             float(rotation.get('z', 0.0)), float(rotation.get('w', 1.0))),
        )

    @staticmethod
    def _quat_mul(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _rotate_vec(q, v):
        x, y, z, w = q
        tx = 2.0 * (y * v[2] - z * v[1])
        ty = 2.0 * (z * v[0] - x * v[2])
        tz = 2.0 * (x * v[1] - y * v[0])
        return (
            v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx),
        )

    # ------------------------------------------------------------------ #
    #  Map
    # ------------------------------------------------------------------ #
    def _on_map(self, msg):
        self._latest_map = msg

    def _publish_cached_map(self):
        if self._latest_map is not None:
            self.map_pub.publish(self._dict_to_occupancy_grid(self._latest_map))

    @staticmethod
    def _dict_to_occupancy_grid(d):
        msg = OccupancyGrid()
        msg.header.frame_id = HOST_FRAME_ID
        info = d.get('info', {})
        msg.info.resolution = float(info.get('resolution', 0.0))
        msg.info.width = int(info.get('width', 0))
        msg.info.height = int(info.get('height', 0))
        origin = info.get('origin', {})
        pos = origin.get('position', {})
        ori = origin.get('orientation', {})
        msg.info.origin.position.x = float(pos.get('x', 0.0))
        msg.info.origin.position.y = float(pos.get('y', 0.0))
        msg.info.origin.position.z = float(pos.get('z', 0.0))
        msg.info.origin.orientation.x = float(ori.get('x', 0.0))
        msg.info.origin.orientation.y = float(ori.get('y', 0.0))
        msg.info.origin.orientation.z = float(ori.get('z', 0.0))
        msg.info.origin.orientation.w = float(ori.get('w', 1.0))
        msg.data = [int(v) for v in d.get('data', [])]
        return msg

    # ------------------------------------------------------------------ #
    #  Scan
    # ------------------------------------------------------------------ #
    def _on_scan(self, msg):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = msg.get('header', {}).get('frame_id', '')
        scan.angle_min = float(msg.get('angle_min', 0.0))
        scan.angle_max = float(msg.get('angle_max', 0.0))
        scan.angle_increment = float(msg.get('angle_increment', 0.0))
        scan.time_increment = float(msg.get('time_increment', 0.0))
        scan.scan_time = float(msg.get('scan_time', 0.0))
        scan.range_min = float(msg.get('range_min', 0.0))
        scan.range_max = float(msg.get('range_max', 0.0))
        # JSON은 NaN/Inf를 표현할 수 없어 rosbridge가 그런 값을 null로 보낸다
        # (레이저 스캔에서 무효/범위밖 리딩에 흔함) — NaN으로 되돌려 REP-117 관례를 따른다.
        scan.ranges = [float(v) if v is not None else float('nan') for v in msg.get('ranges', [])]
        scan.intensities = [float(v) if v is not None else float('nan') for v in msg.get('intensities', [])]
        self.scan_pub.publish(scan)

    # ------------------------------------------------------------------ #
    #  Battery
    # ------------------------------------------------------------------ #
    def _on_battery(self, msg):
        self._latest_battery = msg

    def _log_battery_status(self):
        if self._latest_battery is None:
            self.get_logger().warn('⏳ Waiting for battery state data...')
            return

        percentage = float(self._latest_battery.get('percentage', 0.0)) * 100.0
        voltage = float(self._latest_battery.get('voltage', 0.0))
        self.get_logger().info(
            f'{LOG_GREEN}[Battery Status] {percentage:.1f}% | '
            f'{voltage:.2f}V / {MAX_BATTERY_VOLTAGE:.1f}V{LOG_RESET}'
        )

    # ------------------------------------------------------------------ #
    #  InitialPose (host -> robot)
    # ------------------------------------------------------------------ #
    def _on_initialpose(self, msg):
        pose = msg.pose.pose
        self._initialpose_ws_topic.publish({
            'header': {'frame_id': ROBOT_FRAME_ID},
            'pose': {
                'pose': {
                    'position': {
                        'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
                    },
                    'orientation': {
                        'x': pose.orientation.x, 'y': pose.orientation.y,
                        'z': pose.orientation.z, 'w': pose.orientation.w,
                    },
                },
                'covariance': list(msg.pose.covariance),
            },
        })
        self.get_logger().info(
            f'[InitialPose] {HOST_FRAME_ID} -> {ROBOT_FRAME_ID} 전달 '
            f'(x={pose.position.x:.3f}, y={pose.position.y:.3f})'
        )

    # ------------------------------------------------------------------ #
    #  cmd_vel (host -> robot)
    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg):
        # 연속 발행되는 고빈도 토픽이라 매 메시지 로깅은 하지 않는다.
        self._cmd_vel_ws_topic.publish({
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z},
        })

    # ------------------------------------------------------------------ #
    #  mobile/shift_pose, mobile/rotate (move_cmd_node/move_cmd_action 브리지)
    # ------------------------------------------------------------------ #
    def service_callback_shift_pose(self, request, response):
        self.get_logger().info(
            f'{LOG_BLUE}[Shift] 요청 수신 — distance={request.distance:.3f}m, wait={request.wait}{LOG_RESET}')

        if request.distance == 0.0:
            self.get_logger().warn('[Shift] distance=0, 이동 없음')
            self._log_move_result('Shift', False)
            response.successed = False
            return response

        def do_move():
            ok = self._send_move_cmd(MOVE_CMD_SHIFT, request.distance, MOVE_CMD_LINEAR_SPEED)
            self._log_move_result('Shift', ok)
            return ok

        if request.wait:
            response.successed = do_move()
        else:
            threading.Thread(target=do_move, daemon=True).start()
            response.successed = True
        return response

    def service_callback_rotate(self, request, response):
        self.get_logger().info(
            f'{LOG_BLUE}[Rotate] 요청 수신 — theta={math.degrees(request.theta):.1f}°, wait={request.wait}{LOG_RESET}')

        if request.theta == 0.0:
            self.get_logger().warn('[Rotate] theta=0, 회전 없음')
            self._log_move_result('Rotate', False)
            response.successed = False
            return response

        def do_move():
            ok = self._send_move_cmd(MOVE_CMD_ROTATE, request.theta, MOVE_CMD_ANGULAR_SPEED)
            self._log_move_result('Rotate', ok)
            return ok

        if request.wait:
            response.successed = do_move()
        else:
            threading.Thread(target=do_move, daemon=True).start()
            response.successed = True
        return response

    def service_callback_absolute_rotate(self, request, response):
        self.get_logger().info(
            f'{LOG_BLUE}[AbsRotate] 요청 수신 — theta={math.degrees(request.theta):.1f}°, wait={request.wait}{LOG_RESET}')

        if self.current_pose is None:
            self.get_logger().warn('[AbsRotate] 아직 로봇 위치(TF)를 받지 못했습니다.')
            self._log_move_result('AbsRotate', False)
            response.successed = False
            return response

        current_yaw = self._yaw_from_quaternion(self.current_pose.orientation)
        delta = self._normalize_angle(request.theta - current_yaw)

        if abs(delta) < ABS_ROTATE_EPSILON_RAD:
            self.get_logger().info(
                f'[AbsRotate] 이미 목표 각도({math.degrees(request.theta):.1f}°) 근처입니다. 회전 없음')
            self._log_move_result('AbsRotate', True)
            response.successed = True
            return response

        self.get_logger().info(
            f'[AbsRotate] 현재 {math.degrees(current_yaw):.1f}° -> 목표 '
            f'{math.degrees(request.theta):.1f}° (최소회전 {math.degrees(delta):+.1f}°)'
        )

        def do_move():
            ok = self._send_move_cmd(MOVE_CMD_ROTATE, delta, MOVE_CMD_ANGULAR_SPEED)
            self._log_move_result('AbsRotate', ok)
            return ok

        if request.wait:
            response.successed = do_move()
        else:
            threading.Thread(target=do_move, daemon=True).start()
            response.successed = True
        return response

    def _log_move_result(self, label, success):
        color = LOG_GREEN if success else LOG_RED
        outcome = '완료(성공)' if success else '완료(실패)'
        self.get_logger().info(f'{color}[{label}] {outcome}{LOG_RESET}')

    @staticmethod
    def _yaw_from_quaternion(q):
        """쿼터니언에서 Yaw(z축 회전) 값을 추출한다."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        """각도를 -pi ~ pi 사이로 정규화한다 (최소 회전 방향을 얻기 위함)."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _send_move_cmd(self, command_type, goal_value, speed):
        """MoveCmd 액션을 보내고 완료(SUCCEEDED/그 외)될 때까지 블로킹한다."""
        if not self.ros.is_connected:
            self.get_logger().error('rosbridge(websocket)에 연결돼 있지 않습니다.')
            return False

        timeout_time = max(MOVE_CMD_MIN_TIMEOUT_SEC, abs(goal_value / speed))
        # move_cmd_node/action/MoveCmd의 timeout_time은 int32 필드라 float을 보내면
        # rosbridge가 "requires a int32 ... but got a <class 'float'>"로 거부한다.
        goal = Goal({
            'command_type': command_type,
            'goal': float(goal_value),
            'speed': float(speed),
            'timeout_time': int(math.ceil(timeout_time)),
        })

        done_event = threading.Event()
        final_status = {'value': None}

        def on_settled(payload):
            final_status['value'] = payload.get('status')
            done_event.set()

        self._move_cmd_client.send_goal(goal, on_settled, lambda _fb: None, on_settled)

        if not done_event.wait(timeout_time + MOVE_CMD_WAIT_MARGIN_SEC):
            self.get_logger().error('[MoveCmd] 액션 완료 대기 타임아웃')
            return False

        return final_status['value'] == GoalStatus.SUCCEEDED

    # ------------------------------------------------------------------ #
    #  navigate_to_pose
    # ------------------------------------------------------------------ #
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'{LOG_BLUE}[NavigateToPose] 요청 수신 (frame: {goal_request.pose.header.frame_id}){LOG_RESET}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[NavigateToPose] Cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = NavigateToPose.Result()

        if not self.ros.is_connected:
            self.get_logger().error(
                f'{LOG_RED}[NavigateToPose] 완료(실패) — rosbridge(websocket)에 연결돼 있지 않습니다.{LOG_RESET}')
            goal_handle.abort()
            return result

        goal = Goal({
            'pose': self._pose_stamped_to_dict(goal_handle.request.pose, ROBOT_FRAME_ID),
            'behavior_tree': getattr(goal_handle.request, 'behavior_tree', ''),
        })

        done_event = threading.Event()
        final_status = {'value': None}

        def on_feedback(feedback):
            fb = NavigateToPose.Feedback()
            pose = feedback.get('current_pose', {}).get('pose', {})
            p = pose.get('position', {})
            o = pose.get('orientation', {})
            fb.current_pose.header.frame_id = HOST_FRAME_ID
            fb.current_pose.pose.position.x = float(p.get('x', 0.0))
            fb.current_pose.pose.position.y = float(p.get('y', 0.0))
            fb.current_pose.pose.position.z = float(p.get('z', 0.0))
            fb.current_pose.pose.orientation.x = float(o.get('x', 0.0))
            fb.current_pose.pose.orientation.y = float(o.get('y', 0.0))
            fb.current_pose.pose.orientation.z = float(o.get('z', 0.0))
            fb.current_pose.pose.orientation.w = float(o.get('w', 1.0))
            fb.distance_remaining = float(feedback.get('distance_remaining', 0.0))
            goal_handle.publish_feedback(fb)

        def on_settled(payload):
            # resultback과 errback 둘 다 {'status': GoalStatus, 'values': {...}}
            # 형태를 받는다(roslibpy.core._handle_action_result 참고).
            final_status['value'] = payload.get('status')
            done_event.set()

        goal_id = self._action_client.send_goal(goal, on_settled, on_feedback, on_settled)

        cancel_sent = False
        start = time.monotonic()
        while not done_event.is_set():
            if not cancel_sent and goal_handle.is_cancel_requested:
                cancel_sent = True
                self.get_logger().info('취소 요청 — 로봇 목표를 취소합니다.')
                self._action_client.cancel_goal(goal_id)
            if time.monotonic() - start > GOAL_TIMEOUT_SEC:
                self.get_logger().error('목표 완료 대기 타임아웃')
                break
            time.sleep(0.1)

        status = final_status['value']
        if status == GoalStatus.SUCCEEDED:
            self.get_logger().info(f'{LOG_GREEN}[NavigateToPose] 완료(성공){LOG_RESET}')
            goal_handle.succeed()
        elif status == GoalStatus.CANCELED:
            self.get_logger().warn(f'{LOG_RED}[NavigateToPose] 완료(취소됨){LOG_RESET}')
            goal_handle.canceled()
        else:
            self.get_logger().error(f'{LOG_RED}[NavigateToPose] 완료(실패) — status={status}{LOG_RESET}')
            goal_handle.abort()

        return result

    @staticmethod
    def _pose_stamped_to_dict(pose_stamped, frame_id):
        p = pose_stamped.pose
        return {
            'header': {'frame_id': frame_id},
            'pose': {
                'position': {'x': p.position.x, 'y': p.position.y, 'z': p.position.z},
                'orientation': {
                    'x': p.orientation.x, 'y': p.orientation.y,
                    'z': p.orientation.z, 'w': p.orientation.w,
                },
            },
        }


def main():
    parser = argparse.ArgumentParser(description='Clobot rosbridge/websocket bridge')
    parser.add_argument('--robot-host', default='192.168.11.200',
                         help='로봇 PC의 rosbridge_server 주소')
    parser.add_argument('--robot-port', type=int, default=9090,
                         help='rosbridge_server websocket 포트 (기본 9090)')
    args, _ = parser.parse_known_args()

    rclpy.init()

    ros = roslibpy.Ros(host=args.robot_host, port=args.robot_port)
    ros.run()  # non-blocking: roslibpy가 자체 백그라운드 스레드에서 이벤트 루프를 돈다

    node = ClobotWebsocketBridge(ros, args.robot_host, args.robot_port)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if ros.is_connected:
            ros.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
