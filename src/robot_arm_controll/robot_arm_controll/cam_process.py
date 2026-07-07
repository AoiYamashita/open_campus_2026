import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2
import cv2.aruco as aruco

class Cam_pro(Node):
    def __init__(self):
        super().__init__("cam_process")
        # self.get_logger().info("start process")
        self.cap = cv2.VideoCapture(0)
        self.servo_pub = self.create_subscription(Int32MultiArray,"arm_degs",self.get_degs,1)
        self.servo_pub = self.create_subscription(Float64MultiArray,"hand_pos",self.get_pos,1)

        self.timer = self.create_timer(0.1,self.cb)
        self.parameters = aruco.DetectorParameters_create()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)

        self.mtx = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/mtx.npy")
        self.dist = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/dist.npy")
        self.marker_length = 0.02 # [m]

        self.arm_args = [0,0]
        self.arm_pos = [0,0,0]

    def get_degs(self,msg):
        degs = msg.data
        self.arm_args[0] = np.radians(degs[0]-90)
        self.arm_args[1] = np.radians(degs[1]+degs[2]+degs[3]-180)
    def get_pos(self,msg):
        pos = msg.data
        self.arm_pos = np.array(pos[0:3])
    def cb(self):
        ret, frame = self.cap.read()
        if ret == True:

            h, w = frame.shape[:2]
            newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))

            # Undistort the image
            frame = cv2.undistort(frame, self.mtx, self.dist, None, newCameraMatrix)


            corners, ids, rejected_img_points = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                # マーカーに枠とIDを描画
                aruco.drawDetectedMarkers(frame, corners, ids)
                for corner in corners:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.marker_length, self.mtx, self.dist)

                    xyz = tvec[0][0]
                    xyz[1] *= -1
                    
                    Sy = np.sin(self.arm_args[0])
                    Cy = np.cos(self.arm_args[0])

                    Sp = np.sin(self.arm_args[1])
                    Cp = np.cos(self.arm_args[1])

                    Rz = np.array([
                        [Cy, -Sy, 0],
                        [Sy,  Cy, 0],
                        [ 0,   0, 1]
                    ])

                    Ry = np.array([
                        [Cp, 0, Sp],
                        [0,  1, 0],
                        [-Sp,0,Cp]
                    ])

                    R_arm = Rz @ Ry

                    W_xyz = 1000*R_arm@xyz+self.arm_pos
                    self.get_logger().info(f"{W_xyz}")
                    # self.get_logger().info(f"{self.arm_pos}")


            # フレームを表示
            cv2.imshow('Webcam Live', frame)

            # self.get_logger().info(f"{corners}")

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
