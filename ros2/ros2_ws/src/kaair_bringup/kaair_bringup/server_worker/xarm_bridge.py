#!/usr/bin/env python3

"""
xarm_bridge.py
`/arm/init_set`(std_srvs/Trigger) 수신 시 xarm_api 드라이버(ufactory_driver)의
ROS 서비스를 차례로 호출하여 clean_error → motion_enable → set_mode → set_state
순 초기화를 수행한다.
또한 주기적인 타이머를 활용하여 7셀 리튬폴리머 배터리의 전압 및 잔량을 모니터링한다.

예전에는 이 노드가 xarm Python SDK 로 컨트롤러에 직접 붙었다. 지금은
server_worker_loader 가 ufactory_driver 를 항상 같이 띄우므로 컨트롤러로 가는
연결을 드라이버 하나로 모은다. 그래서
  * 초기화 중에 컨트롤러 소켓을 하나 더 열지 않아 드라이버의 리포트 스트림과
    간섭할 여지가 없고,
  * 초기화 결과가 드라이버가 들고 있는 상태(mode/state)에 그대로 반영되며,
  * robot_ip 를 아는 노드가 드라이버 하나로 줄어든다.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from xarm_msgs.srv import Call, SetInt16, SetInt16ById

# 드라이버 서비스는 성공을 ret=0 으로 돌려준다 (SDK 의 code 와 같은 규약).
def _code_ok(code) -> bool:
    return code is None or code == 0


def _import_xarm_api():
    """xarm Python SDK 를 필요할 때만 불러온다.

    배터리 전압은 xarm_api 가 서비스로도 토픽으로도 내주지 않아서
    (RobotMsg 에 전압 필드가 없다) 이 기능만 여전히 SDK 를 쓴다. import 시점에
    SDK 가 로그 디렉터리를 만들기 때문에 그 준비도 여기서 같이 한다.
    """
    log_root = os.path.join(
        os.path.expanduser('~'), '.UFACTORY', 'log', 'xarm', 'sdk')
    try:
        os.makedirs(log_root, exist_ok=True)
    except OSError:
        pass
    from xarm.wrapper import XArmAPI
    return XArmAPI


class XArmBridge(Node):
    def __init__(self):
        super().__init__('xarm_bridge')

        self.declare_parameter(
            'driver_ns',
            '/xarm',
            ParameterDescriptor(
                description='ufactory_driver 의 서비스 네임스페이스. '
                            'server_worker_loader 의 xarm_hw_ns 와 같아야 한다.',
            ),
        )
        self.declare_parameter(
            'service_timeout',
            5.0,
            ParameterDescriptor(
                description='드라이버 서비스 한 건당 대기 시간 [s]. '
                            'motion_enable 은 브레이크 해제까지 수 초가 걸린다.',
            ),
        )
        self.declare_parameter(
            'service_wait_timeout',
            2.0,
            ParameterDescriptor(
                description='드라이버 서비스가 올라오기를 기다리는 시간 [s]',
            ),
        )
        self.declare_parameter('init_mode', 1)
        self.declare_parameter(
            'init_state',
            0,
            ParameterDescriptor(
                description='set_state 값 (0=동작 가능한 motion state 등, SDK 문서 참고)',
            ),
        )
        self.declare_parameter('motion_enable_servo_id', 8)
        self.declare_parameter('motion_enable', True)

        # 배터리 모니터 전용. 초기화 시퀀스는 더 이상 IP 를 쓰지 않으므로
        # 런치에서 넘겨주지 않고, 필요하면 환경변수나 -p 로 준다.
        self.declare_parameter('robot_ip', '')
        self.declare_parameter(
            'is_radian',
            True,
            ParameterDescriptor(
                description='ROS ufactory 드라이버와 동일하게 기본 라디안.',
            ),
        )

        self.driver_ns = (
            self.get_parameter('driver_ns').get_parameter_value().string_value
            .strip().rstrip('/')
        )

        # 초기화 순서대로. clean_error 는 인자가 없어 Call, set_mode/set_state 는
        # data 하나뿐이라 SetInt16, motion_enable 은 servo_id 가 붙어 SetInt16ById.
        self._clean_error_cli = self._create_driver_client(Call, 'clean_error')
        self._motion_enable_cli = self._create_driver_client(
            SetInt16ById, 'motion_enable')
        self._set_mode_cli = self._create_driver_client(SetInt16, 'set_mode')
        self._set_state_cli = self._create_driver_client(SetInt16, 'set_state')

        self.create_service(Trigger, '/arm/init_set', self._on_init_set)
        self.get_logger().info(
            f'xArm Bridge 시작: `/arm/init_set` → {self.driver_ns}/* 서비스 호출')

        self._init_requested = False
        self._executor = None

        # 7셀 리튬 폴리머 배터리 잔량 연산용 파라미터 (단위: V)
        self.V_MAX = 29.4  # 4.2V * 7
        self.V_MIN = 23.1  # 3.3V * 7

        # 2.5초 주기로 배터리 전압을 읽어오는 타이머 생성
        # self.battery_timer = self.create_timer(2.5, self._on_battery_monitor)

    def attach_executor(self, executor):
        """서비스 응답을 기다릴 때 쓸 실행기.

        초기화 시퀀스는 서비스 콜백이 아니라 메인 루프에서 돌기 때문에, 노드를
        이미 돌리고 있는 그 실행기를 그대로 써야 한다. 여기서 새 실행기를 만들면
        같은 노드를 두 실행기가 잡게 된다.
        """
        self._executor = executor

    def _create_driver_client(self, srv_type, name):
        return self.create_client(srv_type, f'{self.driver_ns}/{name}')

    def _resolve_robot_ip(self):
        ip = (
            self.get_parameter('robot_ip')
            .get_parameter_value()
            .string_value
            .strip()
        )
        if ip:
            return ip
        return os.environ.get('XARM_BRIDGE_ROBOT_IP', '').strip()

    # 타이머를 이용해 로봇 제어기 입력 전압 및 퍼센티지 로깅 수행
    def _on_battery_monitor(self):
        robot_ip = self._resolve_robot_ip()
        if not robot_ip:
            return

        is_radian = self.get_parameter('is_radian').get_parameter_value().bool_value

        # 백그라운드 소켓 블로킹을 최소화하기 위해 타임아웃을 짧게 주거나 짧은 커넥션 유지
        arm = None
        try:
            XArmAPI = _import_xarm_api()

            # 상태 모니터링용 인스턴스 생성
            arm = XArmAPI(robot_ip, is_radian=is_radian, do_not_open=True)
            arm.connect()

            if not arm.connected:
                return

            # SDK로부터 실시간 공급 전압(Volt) 획득
            code, volt = arm.get_robot_volt()

            if _code_ok(code):
                # 7S 리튬폴리머 기준 선형 퍼센티지(%) 연산
                if volt >= self.V_MAX:
                    percentage = 100.0
                elif volt <= self.V_MIN:
                    percentage = 0.0
                else:
                    percentage = ((volt - self.V_MIN) / (self.V_MAX - self.V_MIN)) * 100.0

                # ANSI 이스케이프 코드: \033[92m (밝은 초록색) / \033[0m (색상 초기화)
                GREEN_COLOR = "\033[92m"
                RESET_COLOR = "\033[0m"

                self.get_logger().info(
                    f'{GREEN_COLOR}[xArm Battery] Voltage: {volt:.2f}V | '
                    f'Remaining: {percentage:.1f}% (7S Li-Po Status){RESET_COLOR}'
                )
        except Exception:
            # 주행이나 초기화 도중 소켓 간섭 등으로 인한 예외 메시지는 스트리밍 방해를 방지하기 위해 묵음 처리
            pass
        finally:
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # 초기화 시퀀스
    # ------------------------------------------------------------------

    def _run_init_sequence(self):
        init_mode = int(self.get_parameter('init_mode').get_parameter_value().integer_value)
        init_state = int(self.get_parameter('init_state').get_parameter_value().integer_value)
        servo_id = int(
            self.get_parameter('motion_enable_servo_id').get_parameter_value().integer_value
        )
        motion_on = self.get_parameter('motion_enable').get_parameter_value().bool_value

        # 드라이버는 servo_id 를 그대로 SDK 에 넘기고, 8 이 전체 축을 뜻한다.
        # SDK 래퍼에서 None 이 하던 역할을 여기서는 8 이 맡는다.
        if servo_id <= 0:
            servo_id = 8

        self.get_logger().info(
            f'init_set 시작 (드라이버 {self.driver_ns}) '
            f'mode={init_mode} state={init_state} motion_enable={motion_on} servo_id={servo_id}'
        )

        motion_enable_req = SetInt16ById.Request()
        motion_enable_req.id = servo_id
        motion_enable_req.data = 1 if motion_on else 0

        set_mode_req = SetInt16.Request()
        set_mode_req.data = init_mode

        set_state_req = SetInt16.Request()
        set_state_req.data = init_state

        steps = (
            ('clean_error', self._clean_error_cli, Call.Request()),
            ('motion_enable', self._motion_enable_cli, motion_enable_req),
            ('set_mode', self._set_mode_cli, set_mode_req),
            ('set_state', self._set_state_cli, set_state_req),
        )

        for label, client, request in steps:
            if not self._call(label, client, request):
                return

        self.get_logger().info('init_set 시퀀스 완료')

    def _call(self, label, client, request):
        """드라이버 서비스 한 건 호출. 성공하면 True.

        서비스 콜백이 아니라 메인 루프에서 불리므로 응답을 기다리는 동안
        실행기를 직접 돌려도 자기 자신을 막지 않는다.
        """
        wait = self.get_parameter('service_wait_timeout').get_parameter_value().double_value
        if not client.wait_for_service(timeout_sec=wait):
            self.get_logger().error(
                f'{label}: {client.srv_name} 서비스가 없다 '
                f'(ufactory_driver 기동 여부와 config 의 services 설정 확인)')
            return False

        timeout = self.get_parameter('service_timeout').get_parameter_value().double_value
        future = client.call_async(request)
        self._executor.spin_until_future_complete(future, timeout_sec=timeout)

        if not future.done():
            # 응답을 버리지 않으면 다음 호출이 남은 응답을 받을 수 있다.
            client.remove_pending_request(future)
            self.get_logger().error(f'{label}: {timeout:.1f}s 안에 응답 없음')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(f'{label}: 응답 없음 (호출 실패)')
            return False

        if _code_ok(response.ret):
            self.get_logger().info(f'{label}: ret={response.ret}')
            return True

        self.get_logger().error(
            f'{label}: 실패 ret={response.ret} message={response.message}')
        return False

    # request, response 시그니처 필수
    def _on_init_set(self, request, response):
        self._init_requested = True
        response.success = True
        response.message = 'init_set 요청 수신 — 메인 루프에서 드라이버 서비스 시퀀스 실행'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = XArmBridge()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.attach_executor(executor)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

            if not node._init_requested:
                continue

            node._init_requested = False
            node._run_init_sequence()

    except (KeyboardInterrupt, ExternalShutdownException):
        # ros2 launch 가 SIGINT/SIGTERM 을 보내면 rclpy 가 컨텍스트를 먼저 내리고
        # ExternalShutdownException 을 던진다. 정상 종료 경로다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
