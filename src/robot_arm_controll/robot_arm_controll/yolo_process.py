import rclpy
import base64
import simplejpeg

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
import time
from logging import getLogger


class YoloPro(Node):
    def __init__(self):
        super().__init__("yolo_process")
        logger = getLogger('ultralytics')
        logger.disabled = True

        # self.get_logger().info("start process")
        self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
        self.cam_h = 480
        self.cam_w = 640
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.cam_h)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_w)

        self.deg_sub = self.create_subscription(Int32MultiArray,"arm_degs",self.get_degs,1)
        self.servo_sub = self.create_subscription(Float64MultiArray,"hand_pos",self.get_pos,1)

        self.web_pub = self.create_publisher(String,"webimage",10)
        self.hand_pub = self.create_publisher(Float64MultiArray,"hand_order",10)
        self.deg_order_pub = self.create_publisher(Int32MultiArray,"arm_order",10)
        self.model = YOLO("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/best.pt",
                        verbose=False)

        self.timer = self.create_timer(0.03,self.cb)
        self.makeWorldCoord = self.create_timer(0.03,self.image2world)

        self.mtx = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/mtx.npy")
        self.dist = np.load("/root/open_campus_2026/src/robot_arm_controll/robot_arm_controll/dist.npy")
        self.marker_length = 0.02 # [m]

        self.detections = []
        self.arm_args = [0,0]
        self.arm_pos = [0,0,0]

        self.wait_time = 0

        self.book_y = -100

        self.no_waldo_counter = 0

        self.home_state = [90,135,0,0,90,90]
    def search_waldo(self):
        self.no_waldo_counter += 1
        lim = 60
        if self.no_waldo_counter < lim:
            return
        # self.get_logger().info(f"{self.no_waldo_counter}")

        if self.no_waldo_counter % 10 == 0:
            state = self.home_state.copy()
            state[0] = int(90-90*np.cos(np.pi*(self.no_waldo_counter-lim)/100))
            msg = Int32MultiArray()
            msg.data = state
            self.deg_order_pub.publish(msg)

    def get_degs(self,msg):
        degs = msg.data
        self.arm_args[0] = np.radians(degs[0]-90)
        self.arm_args[1] = np.radians(degs[1]+degs[2]+degs[3]+180)
    def get_pos(self,msg):
        pos = msg.data
        self.arm_pos = np.array(pos[0:3])
    def cvtcam2wor(self,x,y,z,Camera_coordinate,args):
        arm_args_0 = args[0]
        arm_args_1 = args[1]
        R_t = np.array([
            [np.cos(arm_args_0),0,-np.sin(arm_args_0)],
            [0                 ,1,                  0],
            [np.sin(arm_args_0),0, np.cos(arm_args_0)],
            ])
        R_p = np.array([
            [1,                 0,                  0],
            [0,np.cos(arm_args_1),-np.sin(arm_args_1)],
            [0,np.sin(arm_args_1), np.cos(arm_args_1)],
        ])

        # cam_hand_delta = np.array([0,0.05,0.115])
        cam_hand_delta = np.array([0,0.0,0.190])

        delta = R_t@R_p@cam_hand_delta
        delta[1] *= -1
        delta *= 1000


        W_xyz = R_t@R_p@Camera_coordinate
        W_xyz[1] *= -1
        W_xyz *= 1000

        W_y_now = W_xyz[1]

        x -= delta[0]
        y -= delta[1]
        z -= delta[2]

        vector_length = (self.book_y - y)/W_y_now

        W_xyz *= vector_length

        W_xyz += np.array([x,y,z])

        self.get_logger().info(f"{W_xyz}")

        return W_xyz
    def image2world(self):


        if self.detections == []:
            self.search_waldo()
            return
        
        self.no_waldo_counter = 0

        Wdetect = []
        cam_forcus_w = self.mtx[0,0]
        cam_forcus_h = self.mtx[1,1]
        for i,arg,pos in self.detections:
            try:
                x,y = float(i[0][0]),float(i[0][1])
                w,h = float(i[0][2]),float(i[0][3])
                x -= 0.5
                y -= 0.5
                x *= self.cam_w
                y *= self.cam_h

                depth = 1

                cam_coord = np.array([x*depth/(cam_forcus_w),y*depth/(cam_forcus_h),depth])

                W_xyz = self.cvtcam2wor(pos[0],pos[1],pos[2],cam_coord,arg)
                if self.wait_time < 0:
                    msg = Float64MultiArray()
                    msg.data = np.array([W_xyz[0],W_xyz[1]+20,W_xyz[2],-45,90,0])
                    self.hand_pub.publish(msg)
                    self.wait_time = 100

            except:
                pass
        self.detections = []
        self.wait_time -= 1

    def cb(self):
        ret, frame = self.cap.read()
        if ret == True:
            frame_d = cv2.undistort(frame, self.mtx, self.dist, None)

            result = self.model(frame_d)
            
            
            for i in result:
                # self.get_logger().info(f"{i.boxes.conf[0]}")
                if len(i.boxes) == 0:
                    continue
                # if i.boxes.conf[0] < 0.5:
                    # continue
                # リアルタイム性が不足
                # self.detectionsにアーム角度，アーム座標を含めるように修正が必要
                self.detections.append((i.boxes.xywhn,self.arm_args.copy(),self.arm_pos.copy()))

            annotated_frame = result[0].plot()
            # self.detections

            # フレームを表示
            cv2.imshow('Webcam Live', annotated_frame)

            # self.get_logger().info(f"send image")
            img_jpeg = simplejpeg.encode_jpeg(annotated_frame, colorspace = "BGR", quality = 50)
            pub_msg = String()
            pub_msg.data = base64.b64encode(img_jpeg).decode()
            self.web_pub.publish(pub_msg)

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
