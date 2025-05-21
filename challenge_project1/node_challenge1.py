################################## Import ######################################

import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry # Pour la position du robot
from sensor_msgs.msg import LaserScan # Pour les données du Laser
from geometry_msgs.msg import Twist # Pour les commandes de mouvement



################################## Class ######################################

class Challenge1_Node(Node):
    def __init__(self):
        super().__init__('wall_follower_node')  # Nom du nœud ROS

        # Souscription aux données LiDAR
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Souscription à la position du robot
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publication des commandes de mouvement
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer pour appeler la boucle de contrôle toutes les 0.2 secondes
        self.create_timer(0.2, self.control_loop)

        # Variables internes
        self.scan_data = None
        self.X_actuel = 0.0                  # Position actuelle sur l'axe X
        self.best_distance = 0.0             # Distance vers le mur le plus loin
        self.best_angle = 0.0                # Angle vers le mur le plus loin

        # Distance minimale de sécurité (0.5 m)
        self.securite_distance = 0.5

    # Callback du LiDAR : analyse les données dans le cône [-90°, +90°]
    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges  # Tableau des distances

        # Plage d'indices correspondant à -90° et +90°
        min_angle = -math.pi / 2
        max_angle = +math.pi / 2

        index_min = int((min_angle - angle_min) / angle_increment)
        index_max = int((max_angle - angle_min) / angle_increment)

        # Recherche de la distance maximale dans le cône frontal
        max_dist = 0.0
        best_idx = index_min
        for i in range(index_min, index_max):
            if 0 <= i < len(ranges):
                d = ranges[i]
                if math.isfinite(d) and d > max_dist:
                    max_dist = d
                    best_idx = i

        # Mise à jour des valeurs optimales
        self.best_distance = max_dist
        self.best_angle = angle_min + best_idx * angle_increment

    # Callback de l'odométrie : enregistre la position X actuelle
    def odom_callback(self, msg):
        self.X_actuel = msg.pose.pose.position.x

    # Boucle principale de contrôle (appelée toutes les 0.2 s)
    def control_loop(self):
        msg = Twist()  # Message de commande

        # Si aucune donnée de LiDAR n’est encore reçue
        if self.best_distance == 0.0:
            self.get_logger().info("En attente des données du LiDAR...")
            return

        # Si le mur est encore loin (> 0.5 m), on avance
        if self.best_distance > self.securite_distance:
            msg.linear.x = 0.2  # Avance constante sur l'axe X

            # Tourne légèrement vers le mur le plus loin
            msg.angular.z = 0.5 * self.best_angle

            self.get_logger().info(
                f"Avance vers angle {math.degrees(self.best_angle):.1f}° (dist={self.best_distance:.2f}m), X={self.X_actuel:.2f}"
            )
        else:
            # Trop proche : on s'arrête
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(
                f"Mur trop proche ({self.best_distance:.2f}m). Stop. X={self.X_actuel:.2f}"
            )

        # Publication de la commande
        self.publisher_cmd_vel.publish(msg)

# Fonction principale
def main(args=None):
    rclpy.init(args=args)  # Initialisation de ROS 2
    node = Challenge1_Node()  # Création du nœud
    rclpy.spin(node)  # Boucle d'exécution (écoute des messages)
    node.destroy_node()  # Destruction propre du nœud
    rclpy.shutdown()  # Fermeture de ROS 2

# Exécution du script
if __name__ == '__main__':
    main()

