import rclpy

from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray

from . import Arm_Lib
import time

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.servo_pub = self.create_publisher(Int32MultiArray,"arm_args",10)
        self.servo_sub = self.create_subscription(Int32MultiArray,"arm_order",self.app_order,1)

        self.timer = self.create_timer(0.1,self.cb)

        # self.arm = Arm_Lib.Arm_Device()

        self.number = 0

        # angle = [90,90,90,90,90,30]
        # for i in range(1,7):
        #     self.arm.Arm_serial_servo_write(i,angle[i-1],10000)
        #     time.sleep(0.02)
    def cb(self):
        # servo_arg_arr = []
        # for i in range(1,7):
        #     arg = self.arm.Arm_serial_servo_read(i)
        #     servo_arg_arr.append(arg)
        msg = Int32MultiArray()
        # msg.data = servo_arg_arr
        msg.data = [self.number]*6
        self.servo_pub.publish(msg)

        self.number += 1
    def app_order(self,msg):
        arr = msg.data
        # servo_time = 1000 # ms
        # for id,i in enumerate(arr):
        #     self.arm.Arm_serial_servo_write(id,i,servo_time)
        #     time.sleep(0.02)
        # time.sleep(servo_time/1000.0)

def main():
    rclpy.init()
    cont = Controller()
    rclpy.spin(cont)
