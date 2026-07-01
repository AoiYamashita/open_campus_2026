import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import Arm_Lib

class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher = self.create_publisher(UInt8,"topic",10)
        self.timer = self.create_timer(0.5,self.cb)
        self.number = 0
    def cb(self):
        msg = UInt8()
        msg.data = self.number
        self.publisher.publish(msg)
        self.number += 1

def main():
    rclpy.init()
    talker = Talker()
    rclpy.spin(talker)
