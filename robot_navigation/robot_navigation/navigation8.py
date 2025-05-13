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
        self.subscription_line = self.create_subscription(
            String,
            "/line_state",
            self.line_callback,10)
        
        self.subscription_tournant = self.create_subscription( 
            String,
            '/tournant_state', 
            self.tournant_callback,
            10)

        self.subscription = self.create_subscription( # recoit des images du turtlebot
            Point,
            '/target', # /image_raw/compressed meilleur qualité
            self.listener_callback,
            10)
        

        self.message = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.width=640 # 640 et 352
        self.height=480 #480  et  288
        self.line_state=""
        self.tournant_state=""
        self.in_tournant_droite= "avant"
        self.in_tournant_gauche="avant_g"

        self.subscription2= self.create_subscription(String,"/node_statusObstacle",self.RecoitNodeStatus,10)
        self.nodeStatusObstacle="Off"




        # Timer qui appelle la fonction toutes les 100ms
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = self.get_clock().now()
        self.last_target=Point()


    def RecoitNodeStatus(self,msg):
        self.nodeStatusObstacle= msg.data
        self.get_logger().info(self.nodeStatusObstacle)
        
    def line_callback(self, msg):
        self.line_state = msg.data
    
    def tournant_callback(self, msg):
        self.tournant_state = msg.data
       
    def listener_callback(self, data):
        self.last_target = data  # On stocke seulement
        # self.get_logger().info(f"[target_callback] Target reçu : x={data.x}")

    def control_loop(self):
        current_time = self.get_clock().now()

        if self.line_state=="red":
            self.get_logger().info("je suis la ligne rouge ")
            # si tournant exterieur de la ligne rouge detecte donc a droite
            if self.tournant_state == "droite":
                if self.in_tournant_droite == "avant":
                    self.in_tournant_droite = "droit"
                    self.phase_start_time = current_time
                    self.get_logger().info(" je vais bouger")

            if self.in_tournant_droite == "droit":
                elapsed = (current_time - self.phase_start_time).nanoseconds / 1e9
                if elapsed < 7.0:
                    self.get_logger().info(" j'avance")
                    self.message.linear.x = 0.04
                    self.message.angular.z = 0.0
                else:
                    self.in_tournant_droite = "tourne"
                    self.phase_start_time = current_time  # reset timer

            elif self.in_tournant_droite == "tourne":
                elapsed = (current_time - self.phase_start_time).nanoseconds / 1e9
                if elapsed < 5:
                    self.get_logger().info(" je tourne")
                    self.message.linear.x = 0.0
                    self.message.angular.z = -0.3
                else:
                    self.in_tournant_droite = "fin"
                    
            elif self.in_tournant_droite == "fin":
                self.get_logger().info("  Fin")
                # self.message.linear.x = 0.0
                # self.message.angular.z = 0.0
                self.tournant_state = ""
                self.in_tournant_droite= "avant" 

            elif self.tournant_state != "droite" :
                self.get_logger().info("je suis la ligne rouge2 ")

                error_x = self.last_target.x - (self.width // 2)
                error_y = self.last_target.y - (self.height // 2)
                #  pour tourner 
                self.message.angular.z= - float(error_x)/100.0 # 100 gaazebo - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
                # pour se déplacer tout droit
                if abs(error_y)>0.01:
                    self.message.linear.x=0.05
                else:
                    self.message.linear.x=0.0

        elif self.line_state=="green":
                #si tournant exterieur de la ligne verte deetcte donc a gauche
            if self.tournant_state == "gauche" and self.line_state=="green":
                if self.in_tournant_gauche == "avant_g":
                    self.in_tournant_gauche = "droit_g"
                    self.phase_start_time = current_time
                    self.get_logger().info(" je vais bouger")

            if self.in_tournant_gauche == "droit_g":
                elapsed = (current_time - self.phase_start_time).nanoseconds / 1e9
                if elapsed < 7.5:
                    self.get_logger().info(" j'avance")
                    self.message.linear.x = 0.04
                    self.message.angular.z = 0.0
                else:
                    self.in_tournant_gauche = "tourne_g"
                    self.phase_start_time = current_time  # reset timer
            
            elif self.in_tournant_gauche == "tourne_g":
                elapsed = (current_time - self.phase_start_time).nanoseconds / 1e9
                if elapsed < 3.5:
                    self.get_logger().info(" je tourne")
                    self.message.linear.x = 0.0
                    self.message.angular.z = +0.05
                else:
                    self.in_tournant_gauche = "fin_g"
            
            elif self.in_tournant_gauche == "fin_g":
                self.get_logger().info("  Fin")
                # self.message.linear.x = 0.0
                # self.message.angular.z = 0.0
                self.tournant_state = ""
                self.in_tournant_gauche= "avant_g" 

            elif self.tournant_state != "gauche" :
        
                error_x = self.last_target.x - (self.width // 2)
                error_y = self.last_target.y - (self.height // 2)
                #  pour tourner 
                self.message.angular.z= - float(error_x)/100.0 # - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
                # pour se déplacer tout droit
                if abs(error_y)>0.01:
                    self.message.linear.x=0.04
                else:
                    self.message.linear.x=0.0

        else:
            error_x = self.last_target.x - (self.width // 2)
            error_y = self.last_target.y - (self.height // 2)
            #  pour tourner 
            self.message.angular.z= - float(error_x)/200.0 # - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
            # pour se déplacer tout droit
            if abs(error_y)>0.01:
                self.message.linear.x=0.04
            else:
                self.message.linear.x=0.0


        self.publisher.publish(self.message)


def main(args=None):

    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()