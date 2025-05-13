################################# Import ####################################

import rclpy

from rclpy.node import Node # Pour pouvoir créer un Node
from sensor_msgs.msg import LaserScan # Importation des types de messages des lasers
from nav_msgs.msg import Odometry # Importation des types de messages de l'odométrie
from geometry_msgs.msg import Twist # Importation des types de messages pour la commande en vitesse
from std_msgs.msg import String # Pour des tests



################################# Class du Node 1 ####################################

class Node1_LaserScan(Node):
    def __init__(self):
        super().__init__('node1')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Mur à -90° : {msg.ranges[0]:.2f} m")
        self.get_logger().info(f"Mur à 0°   : {msg.ranges[360]:.2f} m")
        self.get_logger().info(f"Mur à +90° : {msg.ranges[720]:.2f} m")

################################# Class du Node 2 ####################################

class Node2_Odometry(Node):
    def __init__(self):
        super().__init__('node2')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(
            f"Position actuelle -> x: {position.x:.2f}, y: {position.y:.2f}, z: {position.z:.2f}"
        )



################################# Class du Node 3 ####################################

class Node3_Publisher(Node):
    def __init__(self):
        super().__init__('node3')
        self.publisher = self.create_publisher(String, '/new_topic', 10)
        timer_period = 1.0  # 1 seconde
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Oui"
        self.publisher.publish(msg)
        self.get_logger().info(f"Message publié: '{msg.data}'")



################################# Main ####################################

def main(args=None):
    rclpy.init(args=args)

    node1 = Node1_LaserScan()
    node2 = Node2_Odometry()
    node3 = Node3_Publisher()

    try:
        rclpy.spin_multi_threaded([node1, node2, node3])
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        node3.destroy_node()
        rclpy.shutdown()

# Ajoute ceci tout en bas
if __name__ == '__main__':
    main()


