#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class LifeSupport(Node):
    def __init__(self):
        super().__init__('life_support')
        self.publisher_ = self.create_publisher(Empty, '/aima/remote/heartbeat', 10)
        # 20ms 发一次，确保机器人每 100ms 至少能收到 5 个包，容错率极高
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Life Support Active. Keeping the robot conscious...")

    def timer_callback(self):
        self.publisher_.publish(Empty())

def main():
    rclpy.init()
    rclpy.spin(LifeSupport())
    rclpy.shutdown()

if __name__ == '__main__':
    main()