import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
from . import Arm_Lib
import time

import tkinter as tk

class controll_app(Node):
    def __init__(self):
        super().__init__("cont_app")
        self.servo_sub = self.create_subscription(Int32MultiArray,"arm_args",self.servo_args,1)
        self.servo_pub = self.create_publisher(Int32MultiArray,"arm_order",10)
        
        self.hand_pub = self.create_publisher(Float64MultiArray,"hand_order",10)
        self.hand_sub = self.create_subscription(Float64MultiArray,"hand_pos",self.hand_posision,10)

        self.servo_args_arr = [0]*6
        self.number = 0
        self.reset_flag = True
        self.reset_pos_flag = True
        self.coordinate = [0,0,0,0,0,0] # x,y,z,entry deg,hand deg,open close
        
        self.root = tk.Tk()
        self.root.title("Jetson GUI Controller")
        self.root.geometry("600x450")

        self.text_label = tk.Label(
            self.root,
            text="-"*10,
            font=("Arial", 18, "bold"),
            padx=1,          # Horizontal padding
            pady=5,           # Vertical padding
            relief=tk.RIDGE, bd=5,
        )
        self.text_label.pack(pady=10,anchor="w")

        self.hand_pos_label = tk.Label(
            self.root,
            text="-"*10,
            font=("Arial", 12, "bold"),
            padx=1,          # Horizontal padding
            pady=5,           # Vertical padding
            relief=tk.RIDGE, bd=5,
        )
        self.hand_pos_label.pack(pady=10,anchor="w")

        servo_roles = ["right-left","R1","R2","R3","hand","open close"]

        self.scales = [tk.Scale(
            self.root,
            from_=0,
            to=180,
            orient=tk.HORIZONTAL,
            width=25,
            length=350,
            label=f"Servo {i+1} ( {servo_roles[i]} )",
            command=self.scale_change
        ) for i in range(6)]
        
        for num,i in enumerate(self.scales):
            i.place(x=200, y=num*70+5)

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.after(10, self.spin_ros)
    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(10, self.spin_ros)

    def on_key_press(self,event):# key board controll

        self.get_logger().info(f"Key pressed: {event.keysym}")

        if event.keysym == "w":
            self.coordinate[2] += 10
        if event.keysym == "a":
            self.coordinate[0] -= 10
        if event.keysym == "s":
            self.coordinate[2] -= 10
        if event.keysym == "d":
            self.coordinate[0] += 10
        
        if event.keysym == "q":
            self.coordinate[1] += 10
        if event.keysym[0:5] == "e":
            self.coordinate[1] -= 10

        self.get_logger().info(f"coordinate : {self.coordinate}")
        
        msg = Float64MultiArray()
        msg.data = self.coordinate
        self.hand_pub.publish(msg)

    def hand_posision(self,msg):
        arr = msg.data

        pos_text = "x:{:5.2f}\ry:{:5.2f}\rz:{:5.2f}\r\rEntry Deg:{: 3}\r\rHand Deg:{: 3}\r\rOpen Close:{: 3}".format(arr[0],arr[1],arr[2],arr[3],arr[4],arr[5])
        self.hand_pos_label.config(text=pos_text)

        if self.reset_pos_flag:
            self.coordinate = arr
        else:
            self.coordinate[3:] = arr[3:]
        self.reset_pos_flag = False

    def scale_change(self,value):# scale data pub
        arr = [i.get() for i in self.scales]
        msg = Int32MultiArray()
        msg.data = arr
        self.servo_pub.publish(msg)

    def servo_args(self,msg):# text value,scale reset
        self.servo_args_arr = msg.data
        text_msg = ""
        for num,i in enumerate(self.servo_args_arr):
            text_msg += f"id{num+1} : {i}\n"
        self.text_label.config(text=text_msg[:-1])
        
        if self.reset_flag:
            for num,i in enumerate(self.servo_args_arr):
                self.scales[num].set(i)
        self.reset_flag = False
        



def main():
    rclpy.init()
    app = controll_app()
    app.root.mainloop()
