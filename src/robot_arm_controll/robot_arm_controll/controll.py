import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
from . import Arm_Lib
import time

R1_L = 82.85
R2_L = 82.85
R3_L = 75
HAND = 115.0

class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        self.arm = Arm_Lib.Arm_Device()

        self.servo_deg_arr = [0,0,0,0,0,0]
        self.order_arr = [0,0,0,0,0,0]
        self.initflag = True
        self.number = 0

        self.servo_pub = self.create_publisher(Int32MultiArray,"arm_degs",10)
        self.servo_sub = self.create_subscription(Int32MultiArray,"arm_order",self.app_order,1)
        self.hand_pub = self.create_publisher(Float64MultiArray,"hand_pos",10)
        self.hand_sub = self.create_subscription(Float64MultiArray,"hand_order",self.hand_pos_order,10)

        self.timer = self.create_timer(0.1,self.cb)
        self.co_timer = self.create_timer(0.1,self.arm_controll_pro)


        self.home_state = [90,135,0,0,90,90]

        self.state_sub = self.create_subscription(UInt8,"state",self.get_state,1)
        self.state_flag = 0#1:start 2:stop 3:reset

        # angle = [90,90,90,90,90,30]
        # for i in range(1,7):
        #     self.arm.Arm_serial_servo_write(i,angle[i-1],10000)
        #     time.sleep(0.02)
    def get_state(self,msg):
        self.state_flag = msg.data
        self.get_logger().info(f"{self.state_flag}")
    def cb(self):
        deg_arr = []
        for i in range(1,7):
            deg = self.arm.Arm_serial_servo_read(i)
            if deg is None:
                deg = self.servo_deg_arr[i-1]
            deg_arr.append(deg)
        
        msg = Int32MultiArray()
        msg.data = deg_arr
        # msg.data = [90,0,90,23,102,40]
        # deg_arr = [90,0,90,23,102,40]
        self.servo_pub.publish(msg)

        self.servo_deg_arr = deg_arr.copy()

        msg = Float64MultiArray()
        args = [np.pi*i/180.0 for i in self.servo_deg_arr]
        R = R1_L*np.cos(args[1])+R2_L*np.cos(args[1]+args[2]-np.pi/2.0)+(R3_L+HAND)*np.cos(args[1]+args[2]+args[3]-np.pi)
        Y = R1_L*np.sin(args[1])+R2_L*np.sin(args[1]+args[2]-np.pi/2.0)+(R3_L+HAND)*np.sin(args[1]+args[2]+args[3]-np.pi)
        X = R*np.cos(args[0])
        Z = R*np.sin(args[0])
        msg.data = [X,Y,Z,\
                    (self.servo_deg_arr[1]+self.servo_deg_arr[2]+self.servo_deg_arr[3]-180)%360,\
                    self.servo_deg_arr[4],\
                    self.servo_deg_arr[5]]
        self.hand_pub.publish(msg)
        self.number += 1
    def app_order(self,msg):
        arr = msg.data
        servo_time = 1000 # ms
        for id,i in enumerate(arr):
            self.arm.Arm_serial_servo_write(id+1,i,servo_time)
            time.sleep(0.02)
        time.sleep(np.max([0.05,servo_time/1000.0]))
    def arm_controll_pro(self):
        if self.state_flag == 2:# stop
            return
        if self.state_flag == 3:# reset
            servo_time = 500 # ms

            for id,i in enumerate(self.home_state):
                self.arm.Arm_serial_servo_write(id+1,int(i),int(servo_time))
                time.sleep(0.05)
            time.sleep(servo_time*1.1/1000)
            return

        #start

        if self.initflag:
            return
        arr = self.order_arr.copy()
        x = arr[0]
        y = arr[1]
        z = arr[2]
        entry_deg = -45#arr[3]
        # if entry_deg > 180:
        #     entry_deg = entry_deg-360
        hand_deg = arr[4]
        hand_open = arr[5]
        clip_arg = np.pi/3

        degs = self.servo_deg_arr.copy()
        degs[0] = int(180*np.arctan2(z,x)/np.pi)
        degs[4] = hand_deg
        degs[5] = hand_open

        R = np.sqrt(x**2+z**2)
        Y = y
        
        now_args = np.radians(degs)
        delta = np.zeros(3)
        
        for _ in range(200):
            now_R = np.array([
                R1_L*np.cos(now_args[1]),
                R2_L*np.cos(now_args[1]+now_args[2]-np.pi/2.0),
                (R3_L+HAND)*np.cos(now_args[1]+now_args[2]+now_args[3]-np.pi)
            ])
            
            now_Y = np.array([
                R1_L*np.sin(now_args[1]),
                R2_L*np.sin(now_args[1]+now_args[2]-np.pi/2.0),
                (R3_L+HAND)*np.sin(now_args[1]+now_args[2]+now_args[3]-np.pi)
            ])

            J = np.array([
                [np.sum(now_Y),
                np.sum(now_Y[1:]),
                np.sum(now_Y[2:])],
                [-np.sum(now_R    ),
                -np.sum(now_R[1:]),
                -np.sum(now_R[2:]),],
            ])

            e = np.array([
                R - np.sum(now_R),
                Y - np.sum(now_Y),
            ])

            H = J.T@J

            g = J.T@e

            J_ea = np.array([
                1,1,1
            ])

            e_ea = (np.radians(entry_deg)-(now_args[1]+now_args[2]+now_args[3]-np.pi))

            H_ea = np.dot(J_ea.T,J_ea)
            g_ea = -J_ea.T*e_ea

            a = 10
            b = 10

            alpha = 0#1/(1+np.exp(a*(np.linalg.norm(e)-b)))

            d = - (1-alpha)*np.linalg.solve(H+np.eye(3)*1e-10,g.T)\
                - alpha*np.linalg.solve(H_ea+np.eye(3)*1e-10,g_ea.T)
            
            delta += d
            now_args[1:4] += d

            now_args = np.clip(now_args,0,np.pi)

            clip_delta = delta-np.clip(delta,-clip_arg,clip_arg)

            if np.linalg.norm(clip_delta) > 0:
                break

        delta = np.clip(delta,-clip_arg,clip_arg)

        degs[1:4] += np.degrees(delta)

        self.get_logger().info(f"{degs}")

        max_ddeg = np.max(abs(np.degrees(delta)))

        servo_time = 50*max_ddeg # ms

        for id,i in enumerate(degs):
            self.arm.Arm_serial_servo_write(id+1,int(i),int(servo_time))
            time.sleep(0.05)
        time.sleep(servo_time*1.1/1000)
        
    def hand_pos_order(self,msg):
        self.initflag = False
        self.order_arr = np.array(msg.data)

def main():
    rclpy.init()
    cont = Controller()
    rclpy.spin(cont)
