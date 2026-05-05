#!/usr/bin/env python3
"""
servo_manager.py
────────────────────────────────────────────────────────────────────────
MoveIt Servo on/off 관리 노드.

외부 서비스 콜 한 번으로 servo_node 의 start/stop 을 제어한다.

▸ 서비스
    ~/enable  (std_srvs/SetBool)
        data=true  → /servo_node/start_servo 호출
        data=false → /servo_node/stop_servo  호출

▸ 퍼블리셔
    ~/is_active  (std_msgs/Bool, transient_local)  ← 현재 servo 활성 상태

▸ 데드맨 타이머 (파라미터 deadman_timeout_sec > 0 인 경우)
    servo 가 활성화된 상태에서 ~/watchdog_ping (std_msgs/Empty) 토픽이
    deadman_timeout_sec 이내에 도달하지 않으면 자동으로 servo 를 비활성화한다.
    외부 조이스틱/제어 노드가 죽었을 때 로봇이 멈추지 않는 문제를 방지한다.

사용 예:
    # servo ON
    ros2 service call /servo_manager/enable std_srvs/srv/SetBool '{data: true}'

    # servo OFF
    ros2 service call /servo_manager/enable std_srvs/srv/SetBool '{data: false}'

    # 현재 상태 확인
    ros2 topic echo /servo_manager/is_active
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Bool, Empty


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class ServoManager(Node):
    """
    MoveIt Servo 의 start/stop 서비스를 SetBool 하나로 감싸는 매니저.
    MultiThreadedExecutor 에서 실행해야 서비스 콜백 내부의 비동기 클라이언트
    호출이 데드락 없이 동작한다.
    """

    def __init__(self):
        super().__init__('servo_manager')

        # ── 파라미터 ─────────────────────────────────────────────────────
        self.declare_parameter('servo_node_name',      'servo_node')
        self.declare_parameter('deadman_timeout_sec',  0.0)   # 0 = 비활성

        servo_ns   = self.get_parameter('servo_node_name').value
        self._deadman_timeout = self.get_parameter('deadman_timeout_sec').value

        # ── 상태 ─────────────────────────────────────────────────────────
        self._is_active  = False
        self._state_lock = threading.Lock()

        # ── Callback group (서비스 콜백 안에서 클라이언트 호출 허용) ─────
        self._cbg = ReentrantCallbackGroup()

        # ── servo_node 클라이언트 ─────────────────────────────────────────
        self._start_cli = self.create_client(
            Trigger,
            f'/{servo_ns}/start_servo',
            callback_group=self._cbg,
        )
        self._stop_cli = self.create_client(
            Trigger,
            f'/{servo_ns}/stop_servo',
            callback_group=self._cbg,
        )

        # ── 서비스 서버 ───────────────────────────────────────────────────
        self._enable_srv = self.create_service(
            SetBool,
            '~/enable',
            self._handle_enable,
            callback_group=self._cbg,
        )

        # ── 상태 퍼블리셔 (latched) ────────────────────────────────────────
        self._state_pub = self.create_publisher(Bool, '~/is_active', _LATCHED_QOS)
        self._publish_state()

        # ── 데드맨 watchdog ────────────────────────────────────────────────
        if self._deadman_timeout > 0.0:
            self._last_ping = self.get_clock().now()
            self.create_subscription(
                Empty,
                '~/watchdog_ping',
                self._on_ping,
                10,
                callback_group=self._cbg,
            )
            self._watchdog_timer = self.create_timer(
                self._deadman_timeout / 2.0,
                self._check_deadman,
                callback_group=self._cbg,
            )
            self.get_logger().info(
                f'Deadman watchdog enabled: timeout={self._deadman_timeout:.1f}s  '
                f'ping topic=~/watchdog_ping'
            )

        self.get_logger().info(
            f'ServoManager ready. '
            f'servo_node=/{servo_ns}  '
            f'enable_service=~/enable'
        )

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────

    def _publish_state(self):
        msg = Bool()
        msg.data = self._is_active
        self._state_pub.publish(msg)

    def _call_trigger(self, client: rclpy.node.Client, timeout_sec: float = 5.0):
        """
        비동기 Trigger 클라이언트를 동기적으로 호출한다.
        threading.Event 를 사용하므로 MultiThreadedExecutor 에서 안전하다.
        """
        done   = threading.Event()
        result = [None]

        def _cb(future):
            result[0] = future.result()
            done.set()

        future = client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done.wait(timeout=timeout_sec):
            return None, 'Timeout waiting for servo_node service response.'
        if result[0] is None:
            return None, 'Service call failed (no result).'
        return result[0], result[0].message

    # ── 서비스 핸들러 ─────────────────────────────────────────────────────

    def _handle_enable(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:

        enable = request.data

        with self._state_lock:
            if enable == self._is_active:
                response.success = True
                response.message = (
                    f'Servo already {"active" if enable else "inactive"}.'
                )
                return response

            client     = self._start_cli if enable else self._stop_cli
            action_str = 'start' if enable else 'stop'

        if not client.wait_for_service(timeout_sec=2.0):
            msg = (
                f'/{self.get_parameter("servo_node_name").value}'
                f'/{action_str}_servo service not available. '
                f'Is servo_node running?'
            )
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        trig_result, trig_msg = self._call_trigger(client)

        if trig_result is None:
            response.success = False
            response.message = trig_msg
            self.get_logger().error(f'Servo {action_str} failed: {trig_msg}')
            return response

        with self._state_lock:
            if trig_result.success:
                self._is_active = enable
                if enable and self._deadman_timeout > 0.0:
                    self._last_ping = self.get_clock().now()
                self._publish_state()
                self.get_logger().info(
                    f'Servo {"STARTED" if enable else "STOPPED"}.'
                )
            else:
                self.get_logger().warn(
                    f'Servo {action_str} returned failure: {trig_result.message}'
                )

        response.success = trig_result.success
        response.message = trig_result.message
        return response

    # ── 데드맨 watchdog ────────────────────────────────────────────────────

    def _on_ping(self, _msg: Empty):
        """~/watchdog_ping 수신 시 타임스탬프 갱신."""
        with self._state_lock:
            self._last_ping = self.get_clock().now()

    def _check_deadman(self):
        """타이머: ping 이 timeout 이상 없으면 servo 를 자동 정지."""
        with self._state_lock:
            if not self._is_active:
                return
            elapsed = (self.get_clock().now() - self._last_ping).nanoseconds * 1e-9
            if elapsed < self._deadman_timeout:
                return

        self.get_logger().warn(
            f'Deadman timeout ({self._deadman_timeout:.1f}s) exceeded. '
            f'Stopping servo automatically.'
        )
        # 데드맨 정지: 서비스 핸들러와 동일한 로직으로 stop 호출
        trig_result, trig_msg = self._call_trigger(self._stop_cli)
        with self._state_lock:
            if trig_result and trig_result.success:
                self._is_active = False
                self._publish_state()
            else:
                self.get_logger().error(
                    f'Deadman auto-stop failed: {trig_msg}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ServoManager()
    # MultiThreadedExecutor: 서비스 콜백 안에서 client.call_async 가 실행되므로
    # 단일 스레드로는 데드락이 발생한다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
