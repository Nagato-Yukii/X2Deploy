#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import JointStateArray, JointState, JointCommandArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import time

class TestDog(Node):
    def __init__(self):
        super().__init__('test_dog')
        self.imu_pub = self.create_publisher(Imu, '/aima/hal/imu/torso/state', 10)
        self.leg_pub = self.create_publisher(JointStateArray, '/aima/hal/joint/leg/state', 10)
        
        # 订阅主控的指令，观察它是否变为了 0
        self.cmd_sub = self.create_subscription(
            JointCommandArray, '/aima/hal/joint/leg/command', self.cmd_cb, 10)

        self.timer = self.create_timer(0.002, self.timer_cb) # 500Hz
        self.start_time = time.time()
        self.triggered = False

    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        elapsed = time.time() - self.start_time
        
        imu_msg = Imu()
        imu_msg.header.stamp = now

        # --- 核心模拟逻辑 ---
        if elapsed < 5.0:
            # 前 5 秒：保持完美水平 (w=1, x,y,z=0)
            imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            if not self.triggered:
                self.get_logger().warn("!!! SIMULATING FALL: SETTING PITCH TO 45 DEGREES !!!")
                self.triggered = True
            
            # 5 秒后：模拟 Pitch 倾斜 45 度 (超过 35 度阈值)
            # 欧拉角 (0, 45°, 0) 转四元数约为 (0, 0.383, 0, 0.924)
            imu_msg.orientation = Quaternion(x=0.0, y=0.383, z=0.0, w=0.924)

        self.imu_pub.publish(imu_msg)

        # 同时发布空的关节状态维持主控运行
        leg_msg = JointStateArray()
        for i in range(12):
            leg_msg.joints.append(JointState(name=f"joint_{i}", position=0.0))
        self.leg_pub.publish(leg_msg)

    def cmd_cb(self, msg):
        if self.triggered:
            # 检查收到的第一个关节的 stiffness 是否为 0
            if len(msg.joints) > 0 and msg.joints[0].stiffness == 0.0:
                self.get_logger().info("SUCCESS: Watchdog caught the fall! Stiffness is now 0.0", throttle_duration_sec=0.5)
            else:
                self.get_logger().error("FAILURE: Robot is falling but stiffness is still non-zero!", throttle_duration_sec=0.1)

def main():
    rclpy.init()
    rclpy.spin(TestDog())
    rclpy.shutdown()

if __name__ == '__main__':
    main()