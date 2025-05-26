#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriberNode(Node):
    def __init__(self):
        super().__init__('imu_subscriber_node')
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10
        )
        self.get_logger().info('IMU Subscriber Node Initialized')

    def imu_callback(self, msg):
        self.get_logger().info(f"\n\
            Header: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n\
            Orientation:\n\
                x: {msg.orientation.x}\n\
                y: {msg.orientation.y}\n\
                z: {msg.orientation.z}\n\
                w: {msg.orientation.w}\n\
            Angular Velocity:\n\
                x: {msg.angular_velocity.x}\n\
                y: {msg.angular_velocity.y}\n\
                z: {msg.angular_velocity.z}\n\
            Linear Acceleration:\n\
                x: {msg.linear_acceleration.x}\n\
                y: {msg.linear_acceleration.y}\n\
                z: {msg.linear_acceleration.z}\n\
            roll pitch: \n\
                roll_X: {msg.orientation_covariance[1]}\n\
                pitch_Y: {msg.orientation_covariance[0]}\n"
        )
        

def main():
    rclpy.init()
    imu_subscriber_node = ImuSubscriberNode()
    try:
        rclpy.spin(imu_subscriber_node)
    finally:
        imu_subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
