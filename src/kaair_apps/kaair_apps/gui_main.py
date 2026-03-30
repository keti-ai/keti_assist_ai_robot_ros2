import tkinter as tk
from tkinter import ttk, scrolledtext

class KaairGuiLayout:
    def __init__(self, root: tk.Tk, callbacks: dict):
        self.root = root
        # callbacks: {'run': 함수, 'stop': 함수, ...}
        self.callbacks = callbacks
        self._setup_ui()

    def _setup_ui(self):
        self.root.title("KAAIR Robot Control Panel")
        self.root.geometry("500x400")

        frame = ttk.LabelFrame(self.root, text="Robot Commands", padding=10)
        frame.pack(fill=tk.X, padx=10, pady=10)

        # 버튼에 외부에서 넘겨받은 콜백 함수 연결
        ttk.Button(frame, text="RUN ACTION", command=self.callbacks.get('on_run')).pack(side=tk.LEFT, padx=5)
        ttk.Button(frame, text="STOP ROBOT", command=self.callbacks.get('on_stop')).pack(side=tk.LEFT, padx=5)

        self.txt_log = scrolledtext.ScrolledText(self.root, height=10, state='disabled')
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def log(self, message):
        def _update():
            self.txt_log.config(state='normal')
            self.txt_log.insert(tk.END, f" {message}\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state='disabled')
        self.root.after(0, _update)