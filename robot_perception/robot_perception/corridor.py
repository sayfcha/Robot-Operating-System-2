import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Corridor(Node):
    def __init__(self):
        super().__init__('corridor')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        self.cmd = Twist()

        # Flags for obstacle detection
        self.obstacle_too_close = False
        self.obstacle_to_avoid = False
        self.obstacle_direction = None

        # Add the timer here
        self.timer = self.create_timer(0.1, self.control_loop)  # 0.1 second interval

    def lidar_callback(self, msg: LaserScan):
        # Convert ranges to numpy array for easier manipulation
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        angles_deg = np.degrees(angles)

        # Replace inf values with the max range of the sensor
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)

        # Normalize angles to 0–360°
        angles_deg = angles_deg % 360

        # ZONE INDICES (left, right, and front zones)
        center_indices = np.where((angles_deg <= 15) | (angles_deg >= 345))[0]
        left_indices = np.where((angles_deg > 15) & (angles_deg <= 45))[0]
        right_indices = np.where((angles_deg >= 315) & (angles_deg < 345))[0]

        center = ranges[center_indices]
        left = ranges[left_indices]
        right = ranges[right_indices]

        # Filter NaNs from the data
        left = left[~np.isnan(left)]
        right = right[~np.isnan(right)]
        center = center[~np.isnan(center)]

        # Remove garbage readings < 0.05 or less than the sensor minimum range
        center = center[(center > 0.05) & (center >= msg.range_min)]
        left = left[(left > 0.05) & (left >= msg.range_min)]
        right = right[(right > 0.05) & (right >= msg.range_min)]

        # Reset obstacle flags
        self.obstacle_too_close = False
        self.obstacle_to_avoid = False
        self.obstacle_direction = None

        # Emergency stop condition (close object in front)
        if len(center) > 0:
            min_center = np.min(center)
            self.get_logger().debug(f"Min(center): {min_center:.2f}")
            if min_center < 0.17:
                self.obstacle_too_close = True
                self.get_logger().debug(f"Emergency stop triggered.")
                return

        # Obstacle avoidance detection (left or right)
        min_left = np.min(left) if len(left) > 0 else float('inf')
        min_right = np.min(right) if len(right) > 0 else float('inf')

        if min_left < 0.17 or min_right < 0.17:
            self.obstacle_to_avoid = True
            self.obstacle_direction = 'left' if min_left < min_right else 'right'

        self.control_loop()

    def control_loop(self):
        # Create the twist message to send to the robot
        msg = Twist()

        # Obstacle avoidance
        if self.obstacle_to_avoid:
            msg.linear.x = 0.01
            if self.obstacle_direction == 'left':
                msg.angular.z = -0.17  # Turn right
                self.get_logger().info("Obstacle on the left")
            else:
                msg.angular.z = 0.17  # Turn left
                self.get_logger().info("Obstacle on the right")
            self.publisher.publish(msg)
            return

        # If no obstacles, move forward
        msg.linear.x = 0.06
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Corridor()
    rclpy.spin(node)
    rclpy.shutdown()
