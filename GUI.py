#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from orbiter_bt.srv import NextAction

class TaskManagerNode(Node):
    def __init__(self, task_queue, lock):
        super().__init__('task_manager_node')
        self.task_queue = task_queue
        self.lock = lock

        # # Service server for /get_next_action
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
        self.root.title("ROS2 Task Manager")

        self.task_queue = deque()
        self.lock = threading.Lock()

        # Restocking section
        self.create_restocking_section()

        # Retrieval section
        self.create_retrieval_section()

        # Task queue display
        self.create_task_queue_display()

        # Start ROS2 node in a separate thread
        self.start_ros2_thread()

        # Update GUI periodically
        self.update_gui()

    def create_restocking_section(self):
        frame = tk.Frame(self.root)
        frame.pack(pady=10)

        tk.Label(frame, text="Restocking").grid(row=0, column=0, columnspan=2)

        tk.Label(frame, text="Name:").grid(row=1, column=0)
        self.restock_name_entry = tk.Entry(frame)
        self.restock_name_entry.grid(row=1, column=1)

        tk.Label(frame, text="Quantity:").grid(row=2, column=0)
        self.restock_quantity_entry = tk.Entry(frame)
        self.restock_quantity_entry.grid(row=2, column=1)

        tk.Button(frame, text="Restocking", command=self.add_restocking_tasks).grid(
            row=3, column=0, columnspan=2, pady=5)

    def create_retrieval_section(self):
        frame = tk.Frame(self.root)
        frame.pack(pady=10)

        tk.Label(frame, text="Retrieval").grid(row=0, column=0, columnspan=2)

        tk.Label(frame, text="Name:").grid(row=1, column=0)
        self.retrieval_name_entry = tk.Entry(frame)
        self.retrieval_name_entry.grid(row=1, column=1)

        tk.Button(frame, text="Retrieval", command=self.add_retrieval_task).grid(
            row=2, column=0, columnspan=2, pady=5)

    def create_task_queue_display(self):
        frame = tk.Frame(self.root)
        frame.pack(pady=10)

        tk.Label(frame, text="Task Queue").pack()

        self.queue_listbox = tk.Listbox(frame, width=50)
        self.queue_listbox.pack()

    def add_restocking_tasks(self):
        name = self.restock_name_entry.get().strip()
        quantity = self.restock_quantity_entry.get().strip()

        if not name or not quantity.isdigit():
            messagebox.showerror("Input Error", "Please enter a valid name and quantity.")
            return

        quantity = int(quantity)
        with self.lock:
            for _ in range(quantity):
                self.task_queue.append({'type': 'Restocking', 'item': name})
        self.restock_name_entry.delete(0, tk.END)
        self.restock_quantity_entry.delete(0, tk.END)

    def add_retrieval_task(self):
        name = self.retrieval_name_entry.get().strip()

        if not name:
            messagebox.showerror("Input Error", "Please enter a valid name.")
            return

        with self.lock:
            self.task_queue.appendleft({'type': 'Retrieval', 'item': name})
        self.retrieval_name_entry.delete(0, tk.END)

    def update_gui(self):
        with self.lock:
            self.queue_listbox.delete(0, tk.END)
            for task in self.task_queue:
                self.queue_listbox.insert(tk.END, f"{task['type']} - {task['item']}")
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
    root.mainloop()
