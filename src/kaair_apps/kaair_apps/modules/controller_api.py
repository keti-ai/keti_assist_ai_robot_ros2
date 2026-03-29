from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class KaairController:
    def __init__(self, node : Node):
        self.node = node 
        self.ctrl_cb_group = MutuallyExclusiveCallbackGroup()
        
