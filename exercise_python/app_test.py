import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import rclpy
from rclpy.node import Node
import sys
import os

# Import path handling
from kaair_apps.kaair_api import KaairRobotAPI

class KaairGuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("KAAIR Robot Task Manager")
        self.root.geometry("600x450")
        
        # 1. Initialize Robot API (ROS 2)
        try:
            if not rclpy.ok():
                rclpy.init()
            
            # Create a dedicated node for GUI
            self.node = rclpy.node.Node('gui_robot_manager')
            
            # Initialize the integrated API with the node
            self.robot = KaairRobotAPI(self.node) 
            
            # Start ROS 2 Executor in a background thread
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.spin_thread.start()
            
        except Exception as e:
            print(f"FAILED to initialize ROS 2: {e}")
            sys.exit(1)

        # 2. Setup UI Layout
        self._setup_ui()
        
        # 3. Register resource cleanup on close
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        self._log("SUCCESS: GUI App started. Robot connected.")

    def _setup_ui(self):
        """Build tkinter widgets"""
        # --- Control Panel (Top) ---
        control_frame = ttk.LabelFrame(self.root, text="Arm Control", padding=10)
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        # Run Action Button (Empty implementation)
        self.btn_action = ttk.Button(
            control_frame, 
            text="RUN TRAJECTORY", 
            command=self._cb_run_action
        )
        self.btn_action.pack(side=tk.LEFT, padx=5)

        # Cancel Button (Empty implementation)
        self.btn_cancel = ttk.Button(
            control_frame, 
            text="CANCEL MOTION", 
            command=self._cb_cancel_action
        )
        self.btn_cancel.pack(side=tk.LEFT, padx=5)
        
        # --- Status Panel (Middle) ---
        status_frame = ttk.Frame(self.root, padding=5)
        status_frame.pack(fill=tk.X, padx=10)
        
        self.lbl_status = ttk.Label(status_frame, text="Status: IDLE", font=("Helvetica", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT)

        # --- Log Window (Bottom) ---
        log_frame = ttk.LabelFrame(self.root, text="System Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=15, state='disabled', font=("Consolas", 9))
        self.txt_log.pack(fill=tk.BOTH, expand=True)

    # --- Internal Utilities ---
    def _log(self, message):
        """Thread-safe logging to the text window"""
        self.root.after(0, self._unsafe_log_update, message)

    def _unsafe_log_update(self, message):
        """Actual log update (Run on Tk main thread)"""
        now = time.strftime("%H:%M:%S")
        msg = f"[{now}] {message}\n"
        self.txt_log.config(state='normal')
        self.txt_log.insert(tk.END, msg)
        self.txt_log.see(tk.END)
        self.txt_log.config(state='disabled')

    def _update_status(self, text, color="black"):
        self.root.after(0, lambda: self.lbl_status.config(text=f"Status: {text}", foreground=color))

    # --- Button Callback Functions (Placeholders) ---
    def _cb_run_action(self):
        """Callback for Run Action button"""
        self._log("EVENT: 'RUN' button clicked.")
        self._update_status("BUSY (Simulated)", "blue")
        
        # Simulated delay to show status change
        def mock_process():
            time.sleep(2)
            self._log("EVENT: Simulated action finished.")
            self._update_status("IDLE", "black")
            
        threading.Thread(target=mock_process, daemon=True).start()

    def _cb_cancel_action(self):
        """Callback for Cancel Action button"""
        self._log("EVENT: 'CANCEL' button clicked.")
        self._update_status("IDLE", "black")

    # --- Cleanup ---
    def _on_closing(self):
        print("Shutting down...")
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

# --- Entry Point ---
if __name__ == '__main__':
    root = tk.Tk()
    app = KaairGuiApp(root)
    root.mainloop()