import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import sys
import select
import termios
import tty
import tf2_ros
from std_msgs.msg import Header
import math

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Keyboard Control Node Initialized")
        
        # Define speed and turn rates
        self.linear_speed = 0.5  # Linear speed (m/s)
        self.angular_speed = 1.0  # Angular speed (rad/s)

        # Timer to read keyboard input and publish TF
        self.timer = self.create_timer(0.1, self.read_keyboard)

        # Initialize position and orientation for TF broadcasting
        self.x_position = 0.0
        self.y_position = 0.0
        self.orientation = 0.0  # No rotation, straight ahead

    def read_keyboard(self):
        key = self.get_key()
        
        # Create a Twist message to control the robot
        twist = Twist()

        if key == 'up':  # Move forward
            twist.linear.x = self.linear_speed
            self.x_position += self.linear_speed * 0.1 * math.cos(self.orientation) # Update x position
            self.y_position += self.linear_speed * 0.1 * math.sin(self.orientation) # Update x position
        elif key == 'down':  # Move backward
            twist.linear.x = -self.linear_speed
            self.y_position -= self.linear_speed * 0.1 * math.sin(self.orientation)
            self.x_position -= self.linear_speed * 0.1 * math.cos(self.orientation)# Update x position
        elif key == 'left':  # Turn left
            twist.angular.z = self.angular_speed
            self.orientation += self.angular_speed * 0.1  # Update orientation
        elif key == 'right':  # Turn right
            twist.angular.z = -self.angular_speed
            self.orientation -= self.angular_speed * 0.1  # Update orientation
        elif key == 'q':  # Quit
            self.get_logger().info("Exiting keyboard control")
            rclpy.shutdown()

        # Publish the Twist message to /cmd_vel
        self.publisher.publish(twist)

        # --- Publish TF transform ---
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_footprint'  # Global frame
        transform.child_frame_id = 'base_link'  # Robot frame

        transform.transform.translation.x = self.x_position
        transform.transform.translation.y = self.y_position
        transform.transform.translation.z = 0.0  # Flat on the ground

        # Convert orientation to quaternion
        # Assuming rotation is around Z-axis only (2D plane)
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.orientation)

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert euler angles (roll, pitch, yaw) to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def get_key(self):
        # Read a single keypress from the terminal
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x1b':  # Arrow key
                    key = sys.stdin.read(2)
                    if key == '[A':  # Up arrow
                        return 'up'
                    elif key == '[B':  # Down arrow
                        return 'down'
                    elif key == '[D':  # Left arrow
                        return 'left'
                    elif key == '[C':  # Right arrow
                        return 'right'
                if key == 'q':  # Quit on 'q' key
                    return 'q'
                return ''
            else:
                return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    keyboard_control = KeyboardControl()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
