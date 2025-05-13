import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/obstacle_alert', 10)
        self.threshold_distance = 0.2  # 10 cm

    def scan_callback(self, msg):
        # On prend une fenêtre d'angles autour de l'avant (centre du tableau)
        self.get_logger().info('node STOP activé')
        ranges = msg.ranges
    
        min_distance = min(min(msg.ranges[0:30]), min(msg.ranges[-30:])) 
        self.get_logger().info(str(min_distance))
        alert_msg = String()
        if 0.01 < min_distance < self.threshold_distance:
            alert_msg.data = 'obstacle'
            self.get_logger().info("OBSTACLE")
        else :
            alert_msg.data = 'rien'
            self.get_logger().info("RIEN")

        self.publisher.publish(alert_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
