import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from kaair_apps.kaair_api import KaairRobotAPI

class RosHandler:
    def __init__(self, gui_logger=None):
        if not rclpy.ok():
            rclpy.init()
        
        self.node = KaairRobotAPI(config_input='/ros_ws/src/kaair_apps/config/app_params.yaml')
        self.logger = gui_logger # GUI에 로그를 찍기 위한 콜백
        
        # ROS 2 스레드 시작
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    # --- [실제 로봇 제어 바인딩 함수들] ---
    
    def handle_run_action(self):
        """GUI의 RUN 버튼과 바인딩될 함수"""
        if self.logger: self.logger("ROS: 명령 수신 - 동작 시작")
        
        # 예: 로봇 팔 준비 및 이동 (비동기로 실행하여 GUI 프리징 방지)
        def task():
            # self.api.arm.ready()  # 아까 만든 브릿지 서비스 호출 등
            # self.api.arm.move_to_target(...)
            import time
            time.sleep(1)
            if self.logger: self.logger("ROS: 동작 완료")
            
        threading.Thread(target=task, daemon=True).start()

    def handle_stop_robot(self):
        """GUI의 STOP 버튼과 바인딩될 함수埋"""
        if self.logger: self.logger("ROS: 긴급 정지 호출!")
        # self.api.arm.stop()
        # self.api.base.stop()
        
        
import tkinter as tk
from gui_main import KaairGuiLayout

class MainApp:
    def __init__(self):
        self.root = tk.Tk()
        
        # 1. ROS 핸들러 생성 (GUI 로그 함수를 인자로 넘김)
        self.ros = RosHandler(gui_logger=self.log_to_gui)
        
        # 2. GUI 콜백 설정 (GUI 버튼 -> ROS 핸들러 함수 바인딩)
        callbacks = {
            'on_run': self.ros.handle_run_action, # 직접 바인딩
            'on_stop': self.ros.handle_stop_robot  # 직접 바인딩
        }
        
        # 3. 레이아웃 생성
        self.gui = KaairGuiLayout(self.root, callbacks)

    def log_to_gui(self, message):
        """RosHandler가 GUI에 메시지를 보낼 때 사용하는 통로"""
        self.gui.log(message)

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = MainApp()
    app.run()