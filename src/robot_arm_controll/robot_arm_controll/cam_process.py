import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2

class Cam_pro(Node):
    def __init__(self):
        super().__init__("cam_process")
        # self.get_logger().info("start process")
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1,self.cb)

    def cb(self):
        ret, frame = self.cap.read()
        if ret == True:
            # フレームを表示
            cv2.imshow('Webcam Live', frame)

            # 'q'キーが押されたらループから抜ける
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
        else:
            return
        pass

def main():
    rclpy.init()
    cam = Cam_pro()
    rclpy.spin(cam)
    cam.cap.release()
    cv2.destroyAllWindows()
