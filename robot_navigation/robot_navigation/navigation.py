import rclpy
from rclpy.node import Node # pour faire des noeuds 
import numpy as np
from geometry_msgs.msg import Point # pour récupérer un point cible à suivre
from geometry_msgs.msg import Twist # pour publier la position du robot
from geometry_msgs.msg import Vector3

from std_msgs.msg import String
class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.subscription = self.create_subscription( # recoit des images du turtlebot
            Point,
            '/target', # /image_raw/compressed meilleur qualité
            self.listener_callback,
            10)

        self.message = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.width=640    #640  #352 
        self.height=480    #480  #288



        self.subscription2= self.create_subscription(String,"/node_statusObstacle",self.RecoitNodeStatus,10)
        self.nodeStatusObstacle="Off"

    def RecoitNodeStatus(self,msg):
        self.nodeStatusObstacle= msg.data
        self.get_logger().info(self.nodeStatusObstacle)

    def listener_callback(self,data):

        if data.x!=0 or data.y!=0:

            # Calcul distance entre la cible et le centre de l'image
            error_x = data.x - (self.width // 2)
            error_y = data.y - (self.height // 2)

            
            #  pour tourner 
            self.message.angular.z= - float(error_x)/100.0 # - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
            
            # pour se déplacer tout droit
            if abs(error_y)>0.01:
                self.message.linear.x=0.05
            else:
                self.message.linear.x=0.0
        else:
            self.message.linear.x=0.0
            self.message.angular.z=0.0
            
        self.publisher.publish(self.message)




def main(args=None):

    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


