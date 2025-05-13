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

        self.subscription2= self.create_subscription(String,"/node_statusObstacle",self.RecoitNodeStatus,10)
        self.nodeStatusObstacle="Off"


        self.subscription = self.create_subscription( # recoit des images du turtlebot
            Image,
            '/camera/image_raw', # /image_raw/compressed meilleur qualité
            self.listener_callback,
            10)
        
        self.br = CvBridge() # pont entre cv2 (utilisant numpy) et ros2 

        
        self.message=Point(x=0.0,y=0.0,z=0.0)
        self.publisher = self.create_publisher(Point,'/target',10)

        # self.nodeStatusFollow=String(data1="On")
        # self.publisher2= self.create_publisher(String,"node_statusFollow",10)

        self.blue_line_counter = 0
        # on met le seuil de ligne bleue detecté à 1 car il est censé detecté qu'une seule ligne bleue 
        # a partir du moment ou il est passé en etat "done" cad apres apres avoir esquivé l'obstacle 
        self.blue_line_threshold = 1  # nombre de lignes bleues à détecter avant de déclencher le corridor
        self.blue_line_detected = False # etat de base ( utile pour la detection de ligne bleu)
        self.allow_blue_detection = False
        self.done_time = rclpy.time.Time() 
        
   

    def RecoitNodeStatus(self,msg):
        self.nodeStatusObstacle= msg.data
        self.get_logger().info(f"Obstacle {self.nodeStatusObstacle}")

        if msg.data == "Done":
            self.done_time = self.get_clock().now()
            self.nodeStatusObstacle = "Done"
            self.allow_blue_detection = True   # une fois qu'on a depasse l'obstacle on peut detecter lal igne bleu 
                                               # pour pouvoir passer en mode "corridor"

     
    def listener_callback(self, data):
        
        
        # la condition du bas est pour qu'on lance le lane following que 2 secondes apres l'evitement du 2e obstacle 
        
        self.get_logger().info(f"oueoue") 

        if self.nodeStatusObstacle == "Off":
            allowed = True
        elif self.nodeStatusObstacle == "Done" :#and self.done_time is not None:
            #elapsed_time = (self.get_clock().now() - self.done_time).nanoseconds
            #allowed = elapsed_time > 2e9


            allowed=True
            self.allow_blue_detection=True # mtn qu'on a depassé l'obstacle on sait qu'a la prochaine ligne bleu
                                           # il y aura le corridor 
        else:
            allowed = False

        if allowed: 

            self.get_logger().info(f"Suivi de ligne")

            # recevoir la video
            # self.get_logger().info('Receiving video frame')
            # get image 
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
            #  passage en modele HSV, teint, saturation et luminosité
            current_frame_hsv=cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
            # découper l'image pour garder que le bas
            self.height, self.width = current_frame.shape[:2] # 480, 600 px
            current_frame_crop=current_frame[int(1/2*self.height):self.height,:]
            current_frame_hsv_crop=current_frame_hsv[int(1/2*self.height):self.height,:]

            #___ ROUGE ___
            # range pour detecter le rouge
            # lower_red=np.array([170,100,100])
            # upper_red=np.array([185,255,255])
            lower_red1=np.array([170,70,50])
            upper_red1=np.array([180,255,255])
            lower_red2=np.array([0,70,50])
            upper_red2=np.array([10,255,255])
            mask1= cv2.inRange(current_frame_hsv_crop,lower_red1,upper_red1)
            mask2= cv2.inRange(current_frame_hsv_crop,lower_red2,upper_red2)
            mask_red= cv2.bitwise_or(mask1,mask2)
            # masque binaire pour  isoler une couleur
            # mask_red = cv2.inRange(current_frame_hsv_crop,lower_red,upper_red) # parcours chaque pixel de l'image et regarde si cela convient
            # garde juste le résultat
            result_red= cv2.bitwise_and(current_frame_crop,current_frame_crop, mask=mask_red) 
            
            #___ VERT___
            # range pour detecter le vert
            lower_green=np.array([30,40,40])
            upper_green=np.array([90,255,255])
            # lower_green=np.array([82,20,140])
            # upper_green=np.array([95,80,200])
            # masque binaire pour  isoler une couleur
            mask_green = cv2.inRange(current_frame_hsv_crop,lower_green,upper_green) # parcours chaque pixel de l'image et regarde si cela convient
            # garde juste le résultat
            result_green= cv2.bitwise_and(current_frame_crop,current_frame_crop, mask=mask_green) 

            #__BLEU__
            # range pour detecter le bleu 
            lower_blue = np.array([85, 50, 150])
            upper_blue = np.array([110, 255, 255])
            mask_blue=cv2.inRange(current_frame_hsv_crop, lower_blue,upper_blue)


            # calcul le moment du mask red, vert et bleu
            M_red= cv2.moments(mask_red)
            M_green= cv2.moments(mask_green)
            M_blue=cv2.moments(mask_blue)

            test=1


            if M_red['m00'] <0 and M_green['m00']<0 and M_blue['m00'] <0:

                print(" je ne vois rien")

            ##elif self.allow_blue_detection and M_blue['m00'] > 0:  # on active la detection de ligne bleu que qd allow_blue_detection==True 
                #self.blue_line_counter += 1   # on rajoute un compteur de ligne blue
                #self.get_logger().info(f"Ligne bleue détectée ({self.blue_line_counter}/2)")

                #if self.blue_line_counter >= self.blue_line_threshold:  # et il réagis qu'a partir de la 2e ligne blue 
                    #self.get_logger().info("Deuxième ligne bleue détectée. Passage au mode corridor.")
                    #self.nodeStatusObstacle = "Corridor"
                    #return
               ## if not self.blue_line_detected:
                    # Transition de "pas de bleu" à "bleu"
                  ##  self.blue_line_counter += 1
                   ## self.get_logger().info(f"[BLEU] Transition détectée : {self.blue_line_counter}/2")
                   ## self.blue_line_detected = True  # On est en train de voir une ligne bleue

                   ## if self.blue_line_counter >= self.blue_line_threshold:
                    ##    self.get_logger().info("Passage au mode corridor")
                      ##  self.nodeStatusObstacle = "Corridor"
                    ##    return  # stop current processing
            

            
            # si je vois 2 ## a enlever pour recuperer l'ancien code 

            ##if M_blue['m00'] > 0:
               ## self.get_logger().info("Ligne bleue détectée. Passage au mode corridor.")
               ## self.nodeStatusObstacle = "Corridor"
               ## return  # on arrete le suivi de ligne à l'entree du corridor et on passe a l'utilisation du lidar

            if self.allow_blue_detection and M_blue['m00'] > 0:
                self.blue_line_counter += 1
                self.get_logger().info(f"[BLEU] Transition détectée : {self.blue_line_counter}")
                self.blue_line_detected = True  # On est en train de voir une ligne bleue

            if self.blue_line_counter >= self.blue_line_threshold:
                self.get_logger().info("Passage au mode corridor")
                self.nodeStatusObstacle = "Corridor"
                return  # stop current processing

            elif M_red['m00']!=0:
                print(" Je ne vois plus la ligne verte! Je suis la rouge")
    

                cx_red = int(M_red['m10'] / M_red['m00'])  # Centre en x de la ligne rouge
                cy_red = int(M_red['m01'] / M_red['m00'])  # Centre en y de la ligne rouge

                # Estimation d'un nouveau point cible en fonction uniquement de la ligne rouge
                contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    # Prendre le plus grand contour (la ligne rouge)
                    contour = max(contours, key=cv2.contourArea)
                    
                    # Approximation de la courbe rouge par une droite
                    [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

            
                    # Calcul des deux points à partir des coordonnées du vecteur directeur
                    # On choisit un facteur d'échelle pour obtenir une ligne visible
                    line_length = 150  # Longueur de la ligne que nous voulons dessiner
                    pt1_x = int(x0 - line_length * vx)
                    pt1_y = int(y0 - line_length * vy)
                    pt2_x = int(x0 + line_length * vx)
                    pt2_y = int(y0 + line_length * vy)

                    # Dessiner la ligne sur l'image
                    cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2

                    # Calcule le vecteur normal à la droite
                    norm = np.sqrt(vx**2 + vy**2)
                    nx = -vy / norm  # x du vecteur normal
                    ny = vx / norm   # y du vecteur normal

                    if nx>0:
                        nx=-nx
                        ny=-ny
                    # pt1_x = int(x0 - line_length * nx)
                    # pt1_y = int(y0 - line_length * ny)
                    pt1_x=int(x0)
                    pt1_y=int(y0)
                    pt2_x = int(x0 + line_length * nx)
                    pt2_y = int(y0 + line_length * ny)

                    cv2.line(current_frame_crop, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 0, 0), 2)  # Bleu, épaisseur 2


                    # d =300 # longeur de la ligne normale
                    # Calcul du point cible, décalé de 15 pixels vers la gauche (selon la direction de la droite)
                    # cx_target = cx_red - 200  # Décalage du point cible sur l'axe x
                    # cy_target = int(slope * cx_target + intercept)  # Calcul de l'ordonnée correspondante
                    # cx_target = int(cx_red +d* nx)
                    # cy_target = int(cy_red +d * ny)
                    cx_target= pt2_x
                    cy_target=pt2_y
                    # Dessiner le point cible
                    cv2.circle(current_frame_crop, (cx_target, cy_target), 5, (0, 255, 0), 2)  # Vert, épaisseur 2

                    self.message.x=float(cx_target)
                    self.message.y= float(cy_target)
                    self.publisher.publish(self.message)
            
            else:
                self.blue_line_detected = False  # on ne voit plus de bleu

            
        
         
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
        
            





def main(args=None):

    rclpy.init(args=args)
    lane_following = LaneFollowing()
    rclpy.spin(lane_following)
    lane_following.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()