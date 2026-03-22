#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import JointStateArray, JointState, JointCommandArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import math

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # 发布模拟的关节状态和IMU数据
        self.leg_pub = self.create_publisher(JointStateArray, '/aima/hal/joint/leg/state', 10)
        self.imu_pub = self.create_publisher(Imu, '/aima/hal/imu/torso/state', 10)

        # 订阅控制指令以验证输出
        self.cmd_sub = self.create_subscription(
            JointCommandArray, '/aima/hal/joint/leg/command',
            self.cmd_callback, 10)

        # 500Hz发布模拟关节状态及IMU数据（匹配真实频率）
        self.timer = self.create_timer(0.002, self.publish_states)

        self.joint_names = [
            "left_hip_pitch_joint", "right_hip_pitch_joint",
            "left_hip_roll_joint", "right_hip_roll_joint",
            "left_hip_yaw_joint", "right_hip_yaw_joint",
            "left_knee_joint", "right_knee_joint",
            "left_ankle_pitch_joint", "right_ankle_pitch_joint",
            "left_ankle_roll_joint", "right_ankle_roll_joint"
        ]

        self.get_logger().info('Test publisher started')

    def publish_states(self):
        # 容器
        leg_msg = JointStateArray()
        for name in self.joint_names:
            # 实例化一个具体的关节单元并赋模拟值
            single_joint = JointState() 
            single_joint.name = name
            single_joint.position = 0.0
            single_joint.velocity = 0.0
            leg_msg.joints.append(single_joint)
        self.leg_pub.publish(leg_msg)

        # 发布IMU数据（模拟直立）
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # 无旋转
        imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.imu_pub.publish(imu_msg)

    def cmd_callback(self, msg):
        # 打印接收到的控制指令
        self.get_logger().info(f'Received command for {len(msg.joints)} joints', throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
