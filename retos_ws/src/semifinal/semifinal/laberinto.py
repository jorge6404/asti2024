import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
from geometry_msgs.msg import Twist
import time
import math
from semifinal.misfunciones import *
import argparse

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)   # cmd_vel has (m/s , rad/s)

    def publish_velocity(self, velocity):            # velocity -->  tuple = (1, 0) (linear.x, angular.z)
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.angular.z = velocity[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

def main(args=None):
    # Argumentos opcionales
    
    parser = argparse.ArgumentParser(description='Publish velocities to a ROS2 topic.')
    parser.add_argument('-vl', type=float, default=0.3, help='The linear to publish.')
    parser.add_argument('-va', type=float, default=2.0, help='The angular to publish.')
    parser.add_argument('-al', type=float, default=0.01, help='Linear acceleration.')
    parser.add_argument('-aa', type=float, default=0.1 , help='Angular acceleration.')
    args = parser.parse_args()

    rclpy.init()
    print('Los argumentos son:', args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()

    time.sleep(2)
    #Laberinto
    vel_lin = args.vl           # Max vel
    vel_ang = args.va
    
    acc_lin = args.al       # Positiva siempre
    acc_ang = args.aa
    
    
    recto(minimal_publisher, vel_lin, distancia_total=1.8, acc_lin=acc_lin)
    derecha(minimal_publisher, vel_ang, degrees= 90, acc_ang=acc_ang)
    recto(minimal_publisher, vel_lin, distancia_total=0.6, acc_lin=acc_lin)
    derecha(minimal_publisher, vel_ang, degrees=90, acc_ang=acc_ang)
    recto(minimal_publisher, vel_lin, distancia_total=1.4, acc_lin=acc_lin)
    izquierda(minimal_publisher, vel_ang, degrees=90, acc_ang=acc_ang)
    recto(minimal_publisher, vel_lin, distancia_total=0.6, acc_lin=acc_lin)
    izquierda(minimal_publisher, vel_ang, degrees=90, acc_ang=acc_ang)
    recto(minimal_publisher, vel_lin, distancia_total=1.8, acc_lin=acc_lin)
    
    # atras(minimal_publisher, 0.1, 1)

    executor.add_node(minimal_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
