# ball_tracker.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.publisher_ = self.create_publisher(Point, '/target', 10)
        self.subscriber_ = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Ball tracker node started.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # Convertir en HSV pour détection couleur
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Détection balle jaune
        lower_yellow = np.array([20, 70, 70])
        upper_yellow = np.array([40, 255, 255])
        mask_ball = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Détection poteaux rouges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask_goal = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Trouver la balle
        contours, _ = cv2.findContours(mask_ball, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest)
            if radius > 10:
                # Détection valide
                pt = Point()
                pt.x = x   
                pt.y = y 
                pt.z = 0.0
                self.publisher_.publish(pt)
                self.get_logger().info(f"Ball at: ({pt.x:.2f}, {pt.y:.2f})")
        # Pour le debug (affichage)
        cv2.imshow('Ball Tracking', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
