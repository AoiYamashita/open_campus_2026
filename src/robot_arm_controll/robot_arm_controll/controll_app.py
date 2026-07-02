import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from . import Arm_Lib
import time

import tkinter as tk


class controll_app(Node):
    def __init__(self):
        super().__init__("cont_app")
        self.servo_sub = self.create_subscription(Int32MultiArray,"arm_args",self.servo_args,1)
        self.servo_pub = self.create_publisher(Int32MultiArray,"arm_order",10)
        
        self.servo_args_arr = [0]*6
        self.number = 0
        
        self.root = tk.Tk()
        self.root.title("Jetson GUI Controller")
        self.root.geometry("600x450")

        # --- TEXT LABEL ---
        self.text_label = tk.Label(
            self.root,
            text="-"*10,
            font=("Arial", 18, "bold"),
            padx=1,          # Horizontal padding
            pady=5,           # Vertical padding
            relief=tk.RIDGE, bd=5,
        )
        self.text_label.pack(pady=10,anchor="w")

        self.scales = [tk.Scale(
            self.root,
            from_=0,
            to=180,
            orient=tk.HORIZONTAL,
            width=30,
            length=350,
            label=f"Servo {i+1}",
            command=self.scale_change
        ) for i in range(6)]
        
        for num,i in enumerate(self.scales):
            i.place(x=200, y=num*65+20)

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.after(10, self.spin_ros)
    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(10, self.spin_ros)

    def on_key_press(self,event):
        self.get_logger().info(f"Key pressed: {event.keysym}")
        if event.keysym == "Return":
            for num,i in enumerate(self.servo_args_arr):
                self.scales[num].set(i)

    def scale_change(self,value):
        arr = [i.get() for i in self.scales]
        msg = Int32MultiArray()
        msg.data = arr
        self.servo_pub.publish(msg)

    def servo_args(self,msg):
        self.servo_args_arr = msg.data
        text_msg = ""
        for num,i in enumerate(self.servo_args_arr):
            text_msg += f"id{num+1} : {i}\n"
        self.text_label.config(text=text_msg[:-1])
        
        



def main():
    rclpy.init()
    app = controll_app()
    app.root.mainloop()
