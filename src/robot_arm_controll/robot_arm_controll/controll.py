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
R3_L = 79.05
HAND = 50.0

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.servo_pub = self.create_publisher(Int32MultiArray,"arm_args",10)
        self.servo_sub = self.create_subscription(Int32MultiArray,"arm_order",self.app_order,1)
        self.hand_pub = self.create_publisher(Float64MultiArray,"hand_pos",10)
        self.hand_sub = self.create_subscription(Float64MultiArray,"hand_order",self.hand_pos_order,10)

        self.timer = self.create_timer(0.1,self.cb)

        self.arm = Arm_Lib.Arm_Device()

        self.servo_deg_arr = [0,0,0,0,0,0]
        self.number = 0

        # angle = [90,90,90,90,90,30]
        # for i in range(1,7):
        #     self.arm.Arm_serial_servo_write(i,angle[i-1],10000)
        #     time.sleep(0.02)
    def cb(self):
        deg_arr = []
        for i in range(1,7):
            deg = self.arm.Arm_serial_servo_read(i)
            if deg is None:
                deg = 90
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

    def hand_pos_order(self,msg):
        try:
            arr = msg.data
            x = arr[0]
            y = arr[1]
            z = arr[2]
            entry_arg = arr[3]
            hand_arg = arr[4]
            hand_open = arr[5]

            args = self.servo_deg_arr.copy()
            args[0] = int(180*np.arctan2(z,x)/np.pi)

            dr = (R3_L+HAND)*np.cos(np.pi*entry_arg/180)
            dy = (R3_L+HAND)*np.sin(np.pi*entry_arg/180)
            r2 = np.sqrt(x**2+z**2) - dr
            y2 = y - dy

            if np.sqrt(r2**2+y2**2) > R1_L+R2_L:
                st_arg = np.arctan2(y2,r2)
                dr = (R3_L+HAND)*np.cos(st_arg)
                dy = (R3_L+HAND)*np.sin(st_arg)
                r2 = np.sqrt(x**2+z**2) - dr
                y2 = y - dy
                entry_arg = int(180*st_arg/np.pi)
                pass
        
            self.get_logger().info(f"{np.sqrt(x**2+z**2)},{y},{r2},{y2}")

            cos_value = (r2**2+y2**2+R1_L**2-R2_L**2)/(2*R1_L*np.sqrt(r2**2+y2**2))
            if cos_value > 1:
                cos_value = 1
            if cos_value < -1:
                cos_value = -1

            theta1 = np.arccos(cos_value)+np.arctan2(y2,r2)
        
            theta2 = np.arctan2(y2-R1_L*np.sin(theta1),r2-R1_L*np.cos(theta1))-theta1+np.pi/2.0

            args[1] = int(180*theta1/np.pi)
            args[2] = int(180*theta2/np.pi)
            args[3] = int(entry_arg-180*theta1/np.pi-180*theta2/np.pi+180)
            args[4] = hand_arg
            args[5] = hand_open

            max_ddeg = np.max(abs(np.array(args)-np.array(self.servo_deg_arr)))
        
            # self.get_logger().info(f"{args}")
            servo_time = max_ddeg*100 # ms
            for id,i in enumerate(args):
                self.arm.Arm_serial_servo_write(id+1,int(i),int(servo_time))
                time.sleep(0.02)
            time.sleep(np.max([0.05,servo_time/1000.0]))
        except:
            pass

def main():
    rclpy.init()
    cont = Controller()
    rclpy.spin(cont)
