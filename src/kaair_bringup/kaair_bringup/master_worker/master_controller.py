import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        self.prev_buttons = []

        self.subscriber_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)


        self.get_logger().info('Master controller node initialized')


    def joy_callback(self, msg : Joy):
        # 첫 실행 시 리스트 크기 맞추기
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            return

        # 2. 모든 버튼을 순회하며 상태 변화 감지
        for i in range(len(msg.buttons)):
            current_val = msg.buttons[i]
            prev_val = self.prev_buttons[i]

            # 상태가 변했을 때만 로직 실행
            if current_val != prev_val:
                if current_val == 1:
                    self.on_button_pressed(i)
                else:
                    self.on_button_released(i)

        # 3. 전체 버튼 리스트 업데이트 (깊은 복사)
        self.prev_buttons = list(msg.buttons)

    def on_button_pressed(self, index):
        self.get_logger().info(f'Button {index} Pressed!')
        
        # # 특정 버튼 인덱스에 따른 동작 정의
        # if index == 0:  # 예: A 버튼
        #     self.call_switch_service(use_remote=True)
        # elif index == 3: # 예: Y 버튼
        #     self.call_switch_service(use_remote=False)

    def on_button_released(self, index):
        self.get_logger().info(f'Button {index} Released!')


def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()