import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
import time

class YoloPro(Node):
    def __init__(self):
        super().__init__("yolo_process")
        # self.get_logger().info("start process")
        self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
        self.deg_sub = self.create_subscription(Int32MultiArray,"arm_degs",self.get_degs,1)
        self.servo_sub = self.create_subscription(Float64MultiArray,"hand_pos",self.get_pos,1)

        self.hand_pub = self.create_publisher(Float64MultiArray,"hand_order",10)
        self.model = YOLO("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/best.pt")

        self.timer = self.create_timer(0.03,self.cb)

        self.mtx = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/mtx.npy")
        self.dist = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/dist.npy")
        self.marker_length = 0.02 # [m]

        self.arm_args = [0,0]
        self.arm_pos = [0,0,0]

        self.wait_time = 0

    def get_degs(self,msg):
        degs = msg.data
        self.arm_args[0] = np.radians(degs[0]-90)
        self.arm_args[1] = np.radians(degs[1]+degs[2]+degs[3]+180)
    def get_pos(self,msg):
        pos = msg.data
        self.arm_pos = np.array(pos[0:3])
    def cb(self):
        ret, frame = self.cap.read()
        if ret == True:
            result = self.model(frame)
            
            # self.get_logger().info(f"{len(result)}")

            annotated_frame = result[0].plot()

            # フレームを表示
            cv2.imshow('Webcam Live', annotated_frame)

            # self.get_logger().info(f"{corners}")

            # 'q'キーが押されたらループから抜ける
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
            self.wait_time -= 1
        else:
            return
        pass

def main():
    rclpy.init()
    cam = YoloPro()
    rclpy.spin(cam)
    cam.cap.release()
    cv2.destroyAllWindows()
