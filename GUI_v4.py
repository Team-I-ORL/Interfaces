#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from orbiter_bt.srv import NextAction
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk 
from PIL import Image, ImageTk
import subprocess
import threading
from tkinter import scrolledtext
import signal
import sys


class TaskManagerNode(Node):
    def __init__(self, task_queue, lock):
        super().__init__('task_manager_node')
        self.task_queue = task_queue
        self.lock = lock

        # Service server for /get_next_action
        self.get_next_action_srv = self.create_service(
            NextAction, '/get_next_action', self.get_next_action_callback)

        # Subscription to /request_clear_restocking_item
        self.subscription = self.create_subscription(
            String, '/request_clear_restocking_item', self.clear_restocking_callback, 10)
    
    def get_next_action_callback(self, request, response):
        with self.lock:
            if self.task_queue:
                task = self.task_queue.popleft()
                response.next_action = task['type']
                response.item = task['item']
                self.get_logger().info(f"Next action: {task['type']} - {task['item']}")
            else:
                response.next_action = 'Wait'
                response.item = 'None'
                self.get_logger().info("No task in queue, waiting...")
        return response

    def clear_restocking_callback(self, msg):
        item_name = msg.data
        with self.lock:
            while self.task_queue and self.task_queue[0]['type'] == 'Restocking' and self.task_queue[0]['item'] == item_name:
                self.task_queue.popleft()
                self.get_logger().info(f"Cleared restocking task for item: {item_name}")


class GUIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ORB Robotics Task Manager")

        self.task_queue = deque()
        self.lock = threading.Lock()

        # Create main frames
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.right_frame = tk.Frame(self.main_frame)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Add image at the top of the left frame
        self.add_header_image("ORB_Robotics.png", parent=self.left_frame)

        # Restocking section in the left frame
        self.create_restocking_section(parent=self.left_frame)

        # Separator between sections
        ttk.Separator(self.left_frame, orient='horizontal').pack(fill='x', pady=5)

        # Retrieval section
        self.create_retrieval_section(parent=self.left_frame)

        # Separator before Task Queue Display
        ttk.Separator(self.left_frame, orient='horizontal').pack(fill='x', pady=5)

        # Task queue display
        self.create_task_queue_display(parent=self.left_frame)

        # Terminal output display in the right frame
        self.create_terminal_output_display(parent=self.right_frame)

        # Start ROS2 node in a separate thread
        self.start_ros2_thread()

        # Start the external process
        self.start_external_process()

        # Update GUI periodically
        self.update_gui()

        self.auto_scroll = True

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Set up SIGINT handler for Ctrl+C
        signal.signal(signal.SIGINT, self.handle_sigint)
        
    def start_external_process(self):
        def run_process():
            command = (
                "source /opt/ros/humble/setup.bash && "
                "source ../../install/setup.bash && "
                "ros2 run orbiter_bt orbiter_bt_node"
            )
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                shell=True,
                executable='/bin/bash'
            )
            for line in self.process.stdout:
                self.append_output(line)

            self.process.wait()

        threading.Thread(target=run_process, daemon=True).start()

    def on_closing(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()
        self.root.destroy()

    def handle_sigint(self, signum, frame):
        self.on_closing()
        sys.exit(0)
    
    def append_output(self, text):
        self.output_text.insert(tk.END, text)
        if self.auto_scroll:
            self.output_text.see(tk.END)

    def add_header_image(self, image_path, max_width=200, max_height=100, parent=None):
        if parent is None:
            parent = self.root
        image = Image.open(image_path)
        image.thumbnail((max_width, max_height), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(image)
        self.header_label = tk.Label(parent, image=photo)
        self.header_label.image = photo  # Keep a reference
        self.header_label.pack(pady=10)

    def create_restocking_section(self, parent=None):
        if parent is None:
            parent = self.root
        frame = tk.Frame(parent)
        frame.pack(pady=10)

        tk.Label(frame, text="Restocking", font=('Helvetica', 14, 'bold')).grid(row=0, column=0, columnspan=2)

        tk.Label(frame, text="Quantity:", font=('Helvetica', 12)).grid(row=1, column=0)
        self.restock_quantity_scale = tk.Scale(frame, from_=1, to=5, orient=tk.HORIZONTAL, length=200)
        self.restock_quantity_scale.set(1)
        self.restock_quantity_scale.grid(row=1, column=1, columnspan=1)

        objects = ["obj1", "obj2"]
        names = ["Gloves", "Masks"]
        for idx, obj in enumerate(objects):
            tk.Button(frame, text=names[idx], command=lambda o=obj: self.add_restocking_tasks(o),
                      bg='#4B89DC', fg='white', width=12).grid(row=2, column=idx, padx=5, pady=5)

    def create_retrieval_section(self, parent=None):
        if parent is None:
            parent = self.root
        frame = tk.Frame(parent)
        frame.pack(pady=10)

        tk.Label(frame, text="Retrieval", font=('Helvetica', 14, 'bold')).grid(row=0, column=0, columnspan=3)

        objects = ["obj1", "obj2", "obj4"]
        names = ["Gloves", "Masks", "Isopropyl Alcohol"]
        for idx, obj in enumerate(objects):
            tk.Button(frame, text=names[idx], command=lambda o=obj: self.add_retrieval_task(o),
                      bg='#2C3E50', fg='white', width=12).grid(row=1, column=idx, padx=5, pady=5)

    def create_task_queue_display(self, parent=None):
        if parent is None:
            parent = self.root
        frame = tk.Frame(parent)
        frame.pack(pady=10)

        tk.Label(frame, text="Task Queue").pack()

        self.queue_listbox = tk.Listbox(frame, width=40)
        self.queue_listbox.pack()

        # Add Clear Queue button
        tk.Button(frame, text="Clear Queue", command=self.clear_queue, bg='#E74C3C', fg='white').pack(pady=5)
    
    def clear_queue(self):
        with self.lock:
            self.task_queue.clear()
        self.update_gui()

    def create_terminal_output_display(self, parent=None):
        if parent is None:
            parent = self.root
        frame = tk.Frame(parent)
        frame.pack(pady=10)

        tk.Label(frame, text="BT LOG").pack()

        self.output_text = scrolledtext.ScrolledText(frame, width=100, height=30)
        self.output_text.pack()

        self.toggle_scroll_button = tk.Button(frame, text="Disable Scrolling", command=self.toggle_auto_scroll)
        self.toggle_scroll_button.pack(pady=5)

    def toggle_auto_scroll(self):
        self.auto_scroll = not self.auto_scroll
        if self.auto_scroll:
            self.toggle_scroll_button.config(text="Disable Scrolling")
        else:
            self.toggle_scroll_button.config(text="Enable Scrolling")

    def add_restocking_tasks(self, item):
        quantity = self.restock_quantity_scale.get()
        with self.lock:
            for _ in range(quantity):
                self.task_queue.append({'type': 'Restocking', 'item': item})

    def add_retrieval_task(self, item):
        with self.lock:
            self.task_queue.appendleft({'type': 'Retrieval', 'item': item})

    def update_gui(self):
        item_to_name = {"obj1": "Gloves", "obj2": "Masks", "obj4": "Isopropyl Alcohol"}
        with self.lock:
            self.queue_listbox.delete(0, tk.END)
            for task in self.task_queue:
                self.queue_listbox.insert(tk.END, f"{task['type']} - {item_to_name[task['item']]}")
        self.root.after(500, self.update_gui)

    def start_ros2_thread(self):
        def ros2_thread():
            rclpy.init()
            self.node = TaskManagerNode(self.task_queue, self.lock)
            rclpy.spin(self.node)
            self.node.destroy_node()
            rclpy.shutdown()

        threading.Thread(target=ros2_thread, daemon=True).start()


if __name__ == '__main__':
    root = tk.Tk()
    app = GUIApp(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()