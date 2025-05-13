import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10) #lecture du lidar
        self.publisher_status = self.create_publisher(String, "/node_statusObstacle", 10) # publier etat on/off du noeud 
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10) #publie cmd de mvt 

        self.state = "IDLE"   # etat du robot 
        self.img_size = 600
        self.scale = 500
        self.center = self.img_size // 2
        self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        self.cmd = Twist()    # commande de mvt du robot 

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop) # appelle toutes les 0.1 seconde la fonction control_loop 

    def lidar_callback(self, msg):
        self.front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))  # on prend les distances en face
        self.right = min(msg.ranges[len(msg.ranges)//2 + 30:len(msg.ranges)//2 + 50]) # on approxime la porté de la droite du robot 
        self.ranges = msg.ranges # on les sauvegardes dans self.ranges 
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def control_loop(self):
        current_time = self.get_clock().now()
        status_msg = String()

        if self.state == "IDLE":
            status_msg.data = "Off"
            if hasattr(self, 'front') and self.front < 0.4:
                self.state = "AVOIDING"
                self.start_time = current_time
                self.get_logger().info("Obstacle détecté devant, début de l'évitement")

        elif self.state == "AVOIDING":
            status_msg.data = "On"
            self.turn_left()
            # Tourner à gauche pendant 6 secondes
            if (current_time - self.start_time).nanoseconds >5.8e9:
                self.state = "RETURNING"
                self.start_time = current_time
                self.get_logger().info("Début du retour vers la trajectoire")

        elif self.state == "RETURNING":
            status_msg.data = "On"
            self.turn_right()
            # Tourner à droite pendant 3 secondes
            if (current_time - self.start_time).nanoseconds > 5.9e9:
                self.state = "FOLLOWING"
                self.start_time = current_time
                self.get_logger().info("Obstacle Partiellement Evité, On avance un peu")
            
        elif self.state == "FOLLOWING":
            status_msg.data = "On"
            self.go_forward()
            if (current_time - self.start_time).nanoseconds > 1.5e9:   # il avance tout droit pdt 1.5 secondes
                self.state = "DONE"
                self.get_logger().info("Obstacle Completement Evité. passage au suvi de ligne")

        elif self.state=="DONE":
            status_msg.data = "Done"



        if self.state in ["AVOIDING", "RETURNING","FOLLOWING"]:
            self.publisher_cmd.publish(self.cmd)

        self.publisher_status.publish(status_msg)

        if hasattr(self, 'ranges'):
            self.image = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
            x, y = self.calculPosLidar(self.ranges, self.angle_min, self.angle_increment)
            self.TraceLidar(x, y, (255, 0, 0))
            self.TraceLidar([self.center], [self.center], (0, 0, 255), 5)
            cv2.imshow("LidarScan", self.image)
            cv2.waitKey(1)


    def go_forward(self):     #fonction qui fais avancer tout droit 
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0

    def go_backward(self):     #fonction qui fais reculer 
        self.cmd.linear.x = -0.1
        self.cmd.angular.z = 0.0

    def turn_left(self):         # fonction qui fais tourner a gauche 
        self.cmd.linear.x = 0.1 
        self.cmd.angular.z = 0.43

    def turn_right(self):       # fonction qui fais tourner vers la droite
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = -0.43

    def calculPosLidar(self, listRange, angleMin, angleIncrement):  # on convertis distance et rayons en coordonné x,y
        xl, yl = [], []
        for i, r in enumerate(listRange):
            if not np.isinf(r):
                angle = angleMin + i * angleIncrement
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                px = int(self.center + x * self.scale)
                py = int(self.center - y * self.scale)
                if 0 <= px < self.img_size and 0 <= py < self.img_size:
                    xl.append(px)
                    yl.append(py)
        return xl, yl

    def TraceLidar(self, x, y, color, size=2):  # on dessine des petits points pour chaque mesure du LIDAR 
        for i in range(len(x)):
            cv2.circle(self.image, (x[i], y[i]), size, color, -1)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    try:
        rclpy.spin(obstacle_avoidance)
    except KeyboardInterrupt:
        pass
    obstacle_avoidance.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
