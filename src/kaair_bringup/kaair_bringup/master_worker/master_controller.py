import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        self.prev_buttons = []

        self._cbg = ReentrantCallbackGroup()

        self.subscriber_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10,
            callback_group=self._cbg,
        )

        self._mode_switch_cli = self.create_client(
            SetBool,
            '/controller_mode_switcher/switch_mode',
            callback_group=self._cbg,
        )

        self.get_logger().info('Master controller node initialized')

    # ── 서비스 호출 ──────────────────────────────────────────────────────────

    def _call_switch_mode(self, data: bool):
        if not self._mode_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                '/controller_mode_switcher/switch_mode service not available'
            )
            return

        req = SetBool.Request()
        req.data = data

        future = self._mode_switch_cli.call_async(req)
        future.add_done_callback(
            lambda f: self._on_switch_response(f, 'forward' if data else 'normal')
        )

    def _on_switch_response(self, future, target_mode: str):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(
                    f'Mode switched to [{target_mode}]: {res.message}'
                )
            else:
                self.get_logger().error(
                    f'Mode switch to [{target_mode}] failed: {res.message}'
                )
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')

    # ── 조이스틱 콜백 ────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            return

        for i in range(len(msg.buttons)):
            current_val = msg.buttons[i]
            prev_val = self.prev_buttons[i]

            if current_val != prev_val:
                if current_val == 1:
                    self.on_button_pressed(i)
                else:
                    self.on_button_released(i)

        self.prev_buttons = list(msg.buttons)

    def on_button_pressed(self, index):
        self.get_logger().info(f'Button {index} Pressed!')

        if index == 4:   # Button 5 → FORWARD 모드
            self._call_switch_mode(data=True)
        elif index == 2: # Button 3 → NORMAL 모드
            self._call_switch_mode(data=False)

    def on_button_released(self, index):
        self.get_logger().info(f'Button {index} Released!')


def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
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