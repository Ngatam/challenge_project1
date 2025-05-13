################################# Import ####################################

import rclpy

from rclpy.node import Node # Pour pouvoir créer un Node
from sensor_msgs.msg import LaserScan # Importation des types de messages des lasers
from nav_msgs.msg import Odometry # Importation des types de messages de l'odométrie
from geometry_msgs.msg import Twist # Importation des types de messages pour la commande en vitesse
from std_msgs.msg import String # Pour des tests



################################# Class ####################################

class NodeChallenge1(Node):

    def __init__(self):
        super().__init__('Node_Challenge_1')
        self.subscription_scanner = self.create_subscription(LaserScan , '/scan',  self.slistener_callback_scan, 10) # Node 1 - Subscriber 1 qui va lire des données de type ‘LaserScan’ dans le topic ‘/Scan’ avec le callback callback_scan
        
        self.subscription_position = self.create_subscription(Odometry , '/odom',  self.listener_callback_pos, 10) # Node 2 - Subscriber 2 qui va lire des données de type ‘Odometry’ dans le topic ‘/odm’ avec le callback callback_scan        
        
        self.publisher_deplacement = self.create_publisher(Twist, "/cmd_vel", 10) # Node 3 - Publisher qui va publier un message de type ‘Twist’ dans le topic ‘/cmd_vel 
        timer_period = 0.5  # Période de publication en seconde
        self.timer = self.create_timer(timer_period, self.timer_callback) # Fonction pour la période de publication        
        self.subscription  # Prévient si variable pas utilisée
        self.i = 0 # Initialisation du compteur


    def listener_callback_scan(self, msg):
        # Envoie des données de la lecteur dans le terminale de commande
        self.get_logger().info("J'ai ", msg.ranges[0], " à 0°") 
        self.get_logger().info("J'ai ", msg.ranges[360], " à 90°") 
        self.get_logger().info("J'ai ", msg.ranges[720], " à 180°") 
              
        
    def listener_callback_position(self, msg):
        position = msg.pose.pose.position # Position X,Y,Z actuel du robot
        orientation = msg.pose.pose.orientation # Rotation alpha, beta, gamma, delta actuel du robot
        
        # Assignation pour calculs
        (x_actuel, y_actuel, z_actuel) = (position.x, position.y, position.z)
        (alpha_actuel, beta_actuel, gamma_actuel, delta_actuel) = (orientation.x, orientation.y, orientation.z, orientation.w)
        
        self.get_logger().info("Je suis en ", x_actuel, " sur X") 
        
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i # Création du message qui va être publié
        self.publisher_.publish(msg) # Publication du message sur le topic
        self.get_logger().info('Publishing: "%s"' % msg.data) # Renvoie la publication sur le terminale de commande
        self.i += 1 # Itération pour savoir combien de message sont publié
        
        """   
        def run(self):
            # Supprimer les distances infinis du nuages de points de données des scanner
            # Convertir le nuage de point en positions x,y
            # Aller à la positon x, y en envoyant les commandes en vitesse sur le topic '/cmd_vel' en choisissant celle avec le x tel   que x>x_actuel
        while True:
        
            # Get character from keyboard
            mykey = click.getchar()  # Read the key that have been pushed
            if mykey in self.keycode.keys():
                char = self.keycode[mykey]  # Rename the keys according to the parameters previously defined
           
                # Twist data:
                # Vector3  linear
            	#   float64 x -> message.linear.x
                #   float64 y -> message.linear.y
                #   float64 z -> message.linear.z
                
                # Vector3  angular
                #	float64 x -> message.angular.x
                #	float64 y -> message.angular.y
                #	float64 z -> message.angular.z

                if char == 'up':    # UP key
                    linear = float(1)
                    angular = float(0)
                    
                if char == 'down':  # DOWN key
                    linear = float(-1)
                    angular = float(0)
                    
                if char == 'right':  # RIGHT key
                    linear = float(0)
                    angular = float(-1)
                   
                if char == 'left':  # LEFT key               
                    linear = float(0)
                    angular = float(1)
                    
                if char == 'stop':  # S key              
                    linear = float(0)
                    angular = float(0)        
                    
                if char == 'quit':  # Q key
                    return False
                    
                self.get_logger().info(f"\n Key pressed: {char}")
                self.message.linear.x = linear * self.linear_scale
                self.message.angular.z = angular * self.angular_scale

                self.publisher_.publish(self.message)

    """

################################# Main ####################################

def main(args=None):
    print("Node 1 lancé")
    rclpy.init(args=args)

    node1 = NodeChallenge1()

    rclpy.spin(node1)

    
    # Run keyboard reading and corresponding publications
    teleop_node.run()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node1.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

