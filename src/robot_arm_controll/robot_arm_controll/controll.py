import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from . import Arm_Lib

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.arm = Arm_Lib.Arm_Device()
        # self.publisher = self.create_publisher(UInt8,"topic",10)
        self.timer = self.create_timer(3.0,self.cb)
        self.number = 0
    def cb(self):
        # msg = UInt8()
        # msg.data = self.number
        # self.publisher.publish(msg)
        if self.number%2 == 0:
            self.arm.Arm_serial_servo_write(6, 90, 500)
        else:
            self.arm.Arm_serial_servo_write(6, 30, 500)
        self.number += 1

def main():
    rclpy.init()
    cont = Controller()
    rclpy.spin(cont)
