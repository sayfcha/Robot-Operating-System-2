import rclpy
from rclpy.node import Node # pour faire des noeuds 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # bibliothèque de opencv
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import String

# from std.msgs.msg import Bool
class LaneFollowing(Node):
    def __init__(self):
        super().__init__('lane_following')

        self.nodeStatusObstacle="Off"
        self.subscription2= self.create_subscription(String,"/node_statusObstacle",self.RecoitNodeStatus,10)

        self.subscription = self.create_subscription( # recoit des images du turtlebot
            Image,
            '/image_raw', # /image_raw/compressed meilleur qualité
            self.listener_callback,
            10)
        self.br = CvBridge() # pont entre cv2 (utilisant numpy) et ros2 


        self.subscription_alert = self.create_subscription(
            String,
            '/obstacle_alert',
            self.alert_callback,
            10
        )


        self.message = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)

        self.where="left"
        self.rp="off"
        
        self.stopUrgence=False
        self.last_target=0

    def RecoitNodeStatus(self,msg):
        self.nodeStatusObstacle= msg.data
        self.get_logger().info(self.nodeStatusObstacle)

    def alert_callback(self, msg: String):
        if msg.data == 'obstacle':
            self.get_logger().info(' Obstacle détecté ! arret urgence')
            self.stopUrgence = True
        else:
            self.stopUrgence= False

    def listener_callback(self, data):

        # Si le node obstacle est inactif
        if self.nodeStatusObstacle== "Off" or self.nodeStatusObstacle == "Done":
            
            self.get_logger().info('Receiving video frame')
            # get image 
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # Passage en modele HSV, teint, saturation et luminosité
            current_frame_hsv=cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
            # Découper l'image pour garder que le bas
            self.height, self.width = current_frame.shape[:2] # 480, 640 px
            current_frame_hsv_crop=current_frame_hsv[int(1/2*self.height):self.height,:]
            self.get_logger().info(f'size{self.height}, {self.width}')

            #___ ROUGE ___
            lower_red1=np.array([170,120,100])
            upper_red1=np.array([180,255,255])
            lower_red2=np.array([0,120,100])
            upper_red2=np.array([10,255,255])
            #___ VERT___
            lower_green=np.array([30,40,40])
            upper_green=np.array([90,255,255])

            # Création des masques
            mask1= cv2.inRange(current_frame_hsv_crop,lower_red1,upper_red1)
            mask2= cv2.inRange(current_frame_hsv_crop,lower_red2,upper_red2)
            mask_red= cv2.bitwise_or(mask1,mask2) 
            mask_green = cv2.inRange(current_frame_hsv_crop,lower_green,upper_green) # parcours chaque pixel de l'image et regarde si cela convient
            maskall1= cv2.inRange(current_frame_hsv,lower_red1,upper_red1)
            maskall2= cv2.inRange(current_frame_hsv,lower_red2,upper_red2)
            mask_allred= cv2.bitwise_or(maskall1,maskall2) 
            mask_allgreen = cv2.inRange(current_frame_hsv,lower_green,upper_green)

            # Barycentres
            M_red= cv2.moments(mask_red)
            M_green= cv2.moments(mask_green)
 
            # Contours 
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
            
            self.draw(current_frame)
            if self.stopUrgence==False:
                    
                if self.detect_rond_point(current_frame):
                    self.rp="on"
                    self.get_logger().info("ROND POINT")
                    if self.where=="left":
                        self.message.angular.z=0.1
                        self.message.linear.x=0.0
                    elif self.where=="right":
                        self.message.angular.z=-0.03
                        self.message.linear.x=0.0
            
        
                elif contours_red and contours_green  : 
                    self.rp="off"
                    cx_red=int(M_red['m10']/M_red['m00']) # centre en x
                    cy_red=int(M_red['m01']/M_red['m00']) # en y
                    
                    cx_green=int(M_green['m10']/M_green['m00']) # centre en x
                    cy_green=int(M_green['m01']/M_green['m00']) # en y
                    self.get_logger().info(f"red y: {cy_red}, green y {cy_green}")

                    
                    # calcul du point à faire suivre
                    cx_target= (cx_red+cx_green)//2
                    cy_target= (cy_red + cy_green)//2
                    # Dessiner
                    cv2.circle(current_frame,(cx_red,cy_red+self.height//2), 5, (0,255,0),2)
                    cv2.circle(current_frame,(cx_green,cy_green+self.height//2), 5, (0,255,0),2)
                    cv2.circle(current_frame,(cx_target,cy_target+self.height//2), 5, (255,255,0),2)

                    if cx_green < cx_red :

                        # if cx_green < 1/20*self.width:
                        #     self.message.angular.z=0.3
                        
                        # else:
                            error_x = float(cx_target) - (self.width // 2)
                            error_y = float(cy_target+ self.height//2) - (self.height // 2)
                            #  pour tourner 
                            self.message.angular.z= - float(error_x)/100.0 # 100 gaazebo - car si error.x >0 la cible est à droite donc angular.z négatif pour tourenr vers la gauche. 100 pour normalisé on peut baisser pour tourner plus vite
                            # pour se déplacer tout droit
                            if abs(error_y)>0.01:
                                self.message.linear.x=0.05
                            else:
                                self.message.linear.x=0.
                            
                    
                    
                elif contours_red and not contours_green and self.rp=="off":
                    cx_red=int(M_red['m10']/M_red['m00']) # centre en x
                    cy_red=int(M_red['m01']/M_red['m00']) # en y
                    cv2.circle(current_frame,(cx_red,cy_red+self.height//2), 5, (0,255,0),2)
                    if cy_red + self.height//2 > 4*self.height/6:
                        # tourner a gauche
                        self.message.angular.z=0.1
                        self.message.linear.x=0.
                    elif cx_red < 1/3*self.width:
                        self.message.angular.z=0.1
                        self.message.linear.x=0.


                elif contours_green and not contours_red and self.rp=="off":
                    cx_green=int(M_green['m10']/M_green['m00']) # centre en x
                    cy_green=int(M_green['m01']/M_green['m00']) # en y
                    cv2.circle(current_frame,(cx_green,cy_green+self.height//2), 5, (0,255,0),2)
                    if cy_green + self.height//2 > 4*self.height/6:
                        # tourner a gauche  
                        self.message.angular.z=-0.1
                        self.message.linear.x=0.
                    # elif cx_green <1/20 * self.width and cy_green + self.height//2 < self.height//2:     # cas spécial pour sortir du rond point               
                    #     self.message.angular.z=0.1
                    #     self.message.linear.x=0.

      
                else: 
                    self.message.linear.x=0.03
                    # self.message.angular.z=0.005
            else:
                self.message.linear.x=0.0
                self.message.angular.z=0.0

            self.publisher.publish(self.message)


            # Afficher 
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
    




    def detect_rond_point(self, current_frame):
        """
        Détecte un rond-point dans la zone centrale (délimitée par un rectangle bleu).
        Retourne True si un rond-point est détecté, sinon False.
        """
        # Convertir l'image en HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Dimensions de l'image
        height, width = current_frame.shape[:2]

        # Définir la même zone que celle utilisée dans draw()
        x1 = int(width * 1/3)
        y1 = int(height * 1/3)
        x2 = int(width * 3/4)
        y2 = int(height * 3/4)

        # Découpe de la zone centrale
        center_crop = current_frame_hsv[y1:y2, x1:x2]

        # Définir les plages de couleurs
        lower_red1 = np.array([170, 70, 50])
        upper_red1 = np.array([180, 255, 255])
        lower_red2 = np.array([0, 70, 50])
        upper_red2 = np.array([10, 255, 255])
        lower_green = np.array([30, 40, 40])
        upper_green = np.array([90, 255, 255])

        # Masques de couleurs
        mask_red = cv2.inRange(center_crop, lower_red1, upper_red1) | cv2.inRange(center_crop, lower_red2, upper_red2)
        mask_green = cv2.inRange(center_crop, lower_green, upper_green)

        # Moments
        M_red = cv2.moments(mask_red)
        M_green = cv2.moments(mask_green)

        # Seuils
        MIN_AREA = 1000
        DIST_THRESHOLD = 100

        if M_red['m00'] > MIN_AREA and M_green['m00'] > MIN_AREA:
            cx_r = int(M_red["m10"] / M_red["m00"])
            cy_r = int(M_red["m01"] / M_red["m00"])
            cx_g = int(M_green["m10"] / M_green["m00"])
            cy_g = int(M_green["m01"] / M_green["m00"])

            distance = np.sqrt((cx_r - cx_g) ** 2 + (cy_r - cy_g) ** 2)

            if distance < DIST_THRESHOLD:
                print(">>> Rond-point détecté dans la zone centrale <<<")
                return True

        return False


    
    def draw(self,current_frame ):
        # Si le node obstacle est inactif
            
           
        # Dimensions de l'image
        height, width = current_frame.shape[:2]  # 480, 640 px

        # Définir la zone centrale pour la détection du rond-point
        top_left = (int(width * 1/3), int(height * 1/3))  # Coin supérieur gauche (centré sur l'image)
        bottom_right = (int(width * 3/4), int(height * 3/4))  # Coin inférieur droit (centré sur l'image)

        # Dessiner un rectangle bleu sur la zone centrale de l'image
        cv2.rectangle(current_frame, top_left, bottom_right, (255, 0, 0), 2)  # Bleu avec épaisseur de 2


def main(args=None):

    rclpy.init(args=args)
    lane_following = LaneFollowing()
    rclpy.spin(lane_following)
    lane_following.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()





