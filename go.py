import math
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaWalkerControl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion


class WalkerFollower(Node):
    def __init__(self):
        super().__init__('walker_follower')

        # Topics
        walker_control_topic = "/carla/ego_vehicle/walker_control_cmd"
        goal_topic = "/goal_pose"
        odometry_topic = "/carla/ego_vehicle/odometry"

        # Publishers and Subscribers
        self.control_publisher = self.create_publisher(CarlaWalkerControl, walker_control_topic, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, odometry_topic, self.odom_callback, 10)

        self.goal_position = None  # 목표 위치
        self.goal_orientation = None  # 목표 방향 (Quaternion)
        self.current_position = None  # 현재 위치
        self.current_orientation = None  # 현재 방향 (Quaternion)

        # Timer for repeated calls to move_towards_goal
        self.timer = self.create_timer(0.1, self.move_towards_goal)  # 0.1초마다 호출

        self.get_logger().info("WalkerFollower initialized.")

    def goal_callback(self, msg):
        """Update the goal position and orientation."""
        self.goal_position = msg.pose.position
        self.goal_orientation = msg.pose.orientation
        self.get_logger().info(f"New goal received: {self.goal_position}, {self.goal_orientation}")

    def odom_callback(self, msg):
        """Update the current position and orientation from odometry."""
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def quaternion_to_euler(self, q):
        """Convert a Quaternion to Euler angles."""
        x, y, z, w = q.x, q.y, q.z, q.w
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def move_towards_goal(self):
        """Move walker towards the goal position and adjust orientation at the goal."""
        if self.goal_position is None or self.current_position is None:
            self.get_logger().info("Goal or current position not yet initialized.")
            return

        # Calculate distance to the goal
        direction_x = self.goal_position.x - self.current_position.x
        direction_y = self.goal_position.y - self.current_position.y
        direction_z = self.goal_position.z - self.current_position.z
        distance = math.sqrt(direction_x ** 2 + direction_y ** 2 + direction_z ** 2)

        # If close to the goal, adjust orientation
        if distance < 0.2:  # 위치에 도달했다고 간주
            self.adjust_orientation()
            return

        # Normalize the direction vector
        magnitude = math.sqrt(direction_x ** 2 + direction_y ** 2 + direction_z ** 2)
        direction_x /= magnitude
        direction_y /= magnitude
        direction_z /= magnitude

        # Create walker control message
        walker_control = CarlaWalkerControl()
        walker_control.direction.x = direction_x
        walker_control.direction.y = direction_y
        walker_control.direction.z = direction_z
        walker_control.speed = 1.0  # Set a constant walking speed

        # Send control command
        self.control_publisher.publish(walker_control)

    def adjust_orientation(self):
        """Adjust the orientation of the walker to match the goal orientation."""
        if self.goal_orientation is None or self.current_orientation is None:
            self.get_logger().info("Orientation data not available.")
            return

        # Convert quaternions to Euler angles
        _, _, current_yaw = self.quaternion_to_euler(self.current_orientation)
        _, _, goal_yaw = self.quaternion_to_euler(self.goal_orientation)

        # Calculate yaw difference
        yaw_diff = goal_yaw - current_yaw

        # Normalize yaw difference to [-pi, pi]
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

        # If orientation is close enough, stop adjustments
        if abs(yaw_diff) < 0.1:  # Allow small error
            self.get_logger().info("Orientation aligned with goal.")
            return

        # Create walker control message for in-place rotation
        walker_control = CarlaWalkerControl()
        walker_control.direction.x = 0.0
        walker_control.direction.y = 0.0
        walker_control.direction.z = 0.0
        walker_control.speed = 0.0  # Stop movement

        # Set rotational velocity (simplified for walker control)
        walker_control.direction.y = 1.0 if yaw_diff > 0 else -1.0  # Rotate left or right

        # Send control command
        self.control_publisher.publish(walker_control)


def main(args=None):
    rclpy.init(args=args)

    walker_follower = WalkerFollower()

    try:
        rclpy.spin(walker_follower)
    except KeyboardInterrupt:
        walker_follower.get_logger().info("Walker follower stopped.")
    finally:
        walker_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
