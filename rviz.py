import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import tf2_ros

import board
import adafruit_bno055
import math
import sys
import select
import termios
import tty

class rviz(Node):
    def __init__(self):
        super().__init__('keyboard_imu_node')

        # ROS 2 Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Keyboard + IMU Rviz Node Initialized")

        # Control state
        self.linear_speed = 0.5
        self.x_position = 0.0
        self.y_position = 0.0
        self.yaw = 0.0  # In radians

        # IMU setup
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Timer callback every 0.1 sec
        self.create_timer(0.1, self.update_loop)

    def update_loop(self):
        self.read_keyboard()
        self.publish_imu()

    def read_keyboard(self):
        key = self.get_key()
        twist = Twist()

        if key == 'up':
            twist.linear.x = self.linear_speed
            self.x_position += self.linear_speed * 0.1 * math.cos(self.yaw)
            self.y_position += self.linear_speed * 0.1 * math.sin(self.yaw)
        elif key == 'down':
            twist.linear.x = -self.linear_speed
            self.x_position -= self.linear_speed * 0.1 * math.cos(self.yaw)
            self.y_position -= self.linear_speed * 0.1 * math.sin(self.yaw)
        elif key == 'q':
            self.get_logger().info("Exiting")
            rclpy.shutdown()

        self.cmd_pub.publish(twist)

    def publish_imu(self):
        try:
            euler = self.sensor.euler
            if euler is None:
                return

            yaw_deg = euler[0] if isinstance(euler[0], (int, float)) else 0.0
            self.yaw = math.radians(yaw_deg)

            # Only yaw used in quaternion
            qx, qy, qz, qw = self.yaw_to_quaternion(self.yaw)

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.imu_pub.publish(imu_msg)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'base_footprint'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = self.x_position
            tf_msg.transform.translation.y = self.y_position
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

        except Exception as e:
            self.get_logger().error(f"IMU error: {e}")

    def yaw_to_quaternion(self, yaw):
        # Converts yaw (in radians) to quaternion with no pitch/roll
        half_yaw = yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return qx, qy, qz, qw

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([fd], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x1b':
                    key = sys.stdin.read(2)
                    if key == '[A':
                        return 'up'
                    elif key == '[B':
                        return 'down'
                if key == 'q':
                    return 'q'
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = rviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
