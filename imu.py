import time
import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import tf2_ros
import math

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.imu_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.get_logger().info('IMU Publisher Node Initialized')

        # Initialize IMU sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Setup the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

    def publish_imu_data(self):
        try:
            # Get orientation in Euler angles (in degrees)
            euler = self.sensor.euler  # (heading, roll, pitch)

            if euler is None:
                return

            roll_deg = euler[1] if isinstance(euler[1], (int, float)) else 0.0
            pitch_deg = euler[2] if isinstance(euler[2], (int, float)) else 0.0
            
            # Clamp pitch between -90 and 90 degrees
            pitch_deg = max(min(pitch_deg, 90.0), -90.0)

            # Convert degrees to radians
            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = 0.0  # No yaw

            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

            # --- Publish IMU data ---
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.imu_publisher.publish(imu_msg)

            # --- Publish TF transform with tilt (only rotation) ---
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_footprint'
            transform.child_frame_id = 'base_link'

            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main():
    rclpy.init()
    imu_publisher_node = ImuPublisherNode()
    rclpy.spin(imu_publisher_node)
    imu_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
