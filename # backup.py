# # backup

# import rclpy
# from rclpy.node import Node  # pour faire des noeuds 
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2  # bibliothèque de opencv
# import numpy as np

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
        
#         # Abonnement à l'image
#         self.subscription = self.create_subscription(
#             Image,
#             '/image_raw',
#             self.listener_callback,
#             10)
        
#         # Publisher pour envoyer des commandes de mouvement
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.br = CvBridge()

#     def listener_callback(self, data):
#         # Recevoir l'image
#         self.get_logger().info('Receiving video frame')
#         current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

#         # Convertir l'image en HSV
#         current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

#         # Découper l'image pour ne garder que le bas
#         height, width = current_frame.shape[:2]
#         current_frame_crop = current_frame[int(1/2 * height):height, :]
#         current_frame_hsv_crop = current_frame_hsv[int(1/2 * height):height, :]

#         # ___ ROUGE ___
#         lower_red = np.array([170, 100, 100])
#         upper_red = np.array([185, 255, 255])
#         mask_red = cv2.inRange(current_frame_hsv_crop, lower_red, upper_red)
#         result_red = cv2.bitwise_and(current_frame_crop, current_frame_crop, mask=mask_red)

#         # ___ VERT ___
#         lower_green = np.array([30, 40, 40])
#         upper_green = np.array([90, 255, 255])
#         mask_green = cv2.inRange(current_frame_hsv_crop, lower_green, upper_green)
#         result_green = cv2.bitwise_and(current_frame_crop, current_frame_crop, mask=mask_green)

#         # Calcul des moments pour le rouge et le vert
#         M_red = cv2.moments(mask_red)
#         M_green = cv2.moments(mask_green)

#         if M_red['m00'] > 0 and M_green['m00'] > 0:
#             # Calcul des barycentres
#             cx_red = int(M_red['m10'] / M_red['m00'])
#             cy_red = int(M_red['m01'] / M_red['m00'])
#             cx_green = int(M_green['m10'] / M_green['m00'])
#             cy_green = int(M_green['m01'] / M_green['m00'])

#             # Dessiner des cercles sur les barycentres
#             cv2.circle(current_frame_crop, (cx_red, cy_red), 5, (0, 255, 0), 2)
#             cv2.circle(current_frame_crop, (cx_green, cy_green), 5, (0, 255, 0), 2)

#             # Calcul du point cible (milieu des barycentres)
#             cx_target = (cx_red + cx_green) // 2
#             cy_target = (cy_red + cy_green) // 2

#             # Dessiner un rectangle autour du point cible
#             cv2.rectangle(current_frame_crop, (cx_target - 5, cy_target - 5),
#                           (cx_target + 5, cy_target + 5), (0, 255, 0), 2)

#             # Calculer l'angle et la distance pour se déplacer
#             # Le robot est centré en (width // 2, height // 2) pour cette image
#             error_x = cx_target - (width // 2)
#             error_y = cy_target - (height // 2)

#             # Le robot doit tourner pour aligner l'angle, et se déplacer vers le point
#             cmd = Twist()

#             # Commande angulaire (rotation du robot autour de Z)
#             cmd.angular.z = -float(error_x) / 100.0  # Rotation en fonction de l'écart horizontal

#             # Commande linéaire (déplacement en ligne droite)
#             if abs(error_x) < 30:  # Se déplacer si l'erreur X est faible
#                 cmd.linear.x = 0.1  # Avancer
#             else:
#                 cmd.linear.x = 0.0  # S'arrêter si on n'est pas bien aligné

#             # Publier la commande de mouvement
#             self.publisher.publish(cmd)

#         # Affichage de l'image avec les annotations
#         cv2.imshow("camera", current_frame_crop)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = ImageSubscriber()
#     rclpy.spin(image_subscriber)
#     image_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# backup








import rclpy
from rclpy.node import Node  # pour faire des noeuds 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2  # bibliothèque de opencv
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Abonnement à l'image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        
        # Publisher pour envoyer des commandes de mouvement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.br = CvBridge()

    def listener_callback(self, data):
        # Recevoir l'image
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convertir l'image en HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Découper l'image pour ne garder que le bas
        height, width = current_frame.shape[:2]
        current_frame_crop = current_frame[int(1/2 * height):height, :]
        current_frame_hsv_crop = current_frame_hsv[int(1/2 * height):height, :]

        # ___ ROUGE ___
        lower_red = np.array([170, 100, 100])
        upper_red = np.array([185, 255, 255])
        mask_red = cv2.inRange(current_frame_hsv_crop, lower_red, upper_red)
        result_red = cv2.bitwise_and(current_frame_crop, current_frame_crop, mask=mask_red)

        # ___ VERT ___
        lower_green = np.array([30, 40, 40])
        upper_green = np.array([90, 255, 255])
        mask_green = cv2.inRange(current_frame_hsv_crop, lower_green, upper_green)
        result_green = cv2.bitwise_and(current_frame_crop, current_frame_crop, mask=mask_green)

        # Calcul des moments pour le rouge et le vert
        M_red = cv2.moments(mask_red)
        M_green = cv2.moments(mask_green)

        if M_green['m00'] == 0:  # Cas où la ligne verte n'est pas détectée
            if M_red['m00'] > 0:
                # Calcul du barycentre de la ligne rouge
                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])

                # Tracer la ligne rouge
                contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    # Prendre le plus grand contour (la ligne rouge)
                    contour = max(contours, key=cv2.contourArea)
                    # Approximation de la courbe rouge par une droite
                    [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                    
                    # Calcul de la direction de la ligne
                    slope = vy / vx  # pente de la ligne
                    intercept = y0 - slope * x0  # ordonnée à l'origine

                    # Calcul du point cible
                    target_x = cx_red - 15  # Décaler le point cible de 15 pixels sur l'axe x (vers la gauche)
                    target_y = int(slope * target_x + intercept)

                    # Dessiner la ligne et la cible
                    cv2.circle(current_frame_crop, (target_x, target_y), 5, (0, 255, 0), 2)
                    cv2.line(current_frame_crop, (x0, y0), (int(x0 + 1000 * vx), int(y0 + 1000 * vy)), (255, 0, 0), 2)

                    # Commande de déplacement
                    cmd = Twist()
                    error_x = target_x - (width // 2)
                    error_y = target_y - (height // 2)

                    # Commande angulaire (rotation)
                    cmd.angular.z = -float(error_x) / 100.0  # Contrôler la rotation du robot

                    # Commande linéaire (avancer ou s'arrêter)
                    if abs(error_x) < 30:
                        cmd.linear.x = 0.1
                    else:
                        cmd.linear.x = 0.0

                    # Publier la commande
                    self.publisher.publish(cmd)

        # Affichage de l'image avec les annotations
        cv2.imshow("camera", current_frame_crop)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

