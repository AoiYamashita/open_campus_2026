import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from . import Arm_Lib
import time

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.arm = Arm_Lib.Arm_Device()
        self.timer = self.create_timer(1.0,self.cb)
        self.number = 0
        # self.arm.Arm_serial_servo_write6_array([90,90,90,90,90,90],10000)
        angle = [90,90,90,90,90,30]
        for i in range(1,7):
            self.arm.Arm_serial_servo_write(i,angle[i-1],10000)
            time.sleep(0.02)
    def cb(self):
        # self.get_logger().info(f"-"*10)

        # for i in range(1,7):
        #     arg = self.arm.Arm_serial_servo_read(i)
        #     self.get_logger().info(f"{i}:{arg}")
        # if self.number%2 == 0:
        #     self.arm.Arm_serial_servo_write6_array([90,0,90,90,0,90],1000)
        # else:
        #     self.arm.Arm_serial_servo_write6_array([90,0,90,90,180,90],1000)
        self.number += 1

def main():
    rclpy.init()
    cont = Controller()
    rclpy.spin(cont)
