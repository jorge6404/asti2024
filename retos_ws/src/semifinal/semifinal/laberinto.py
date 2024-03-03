import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
from geometry_msgs.msg import Twist
import time
import math
from semifinal.misfunciones import *

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
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()

    time.sleep(2)
    #Laberinto
    recto(minimal_publisher, 0.1, 1.8)
    derecha(minimal_publisher, 0.1, 90)
    recto(minimal_publisher, 0.1, 0.6)
    derecha(minimal_publisher, 0.1, 90)
    recto(minimal_publisher, 0.1, 1.4)
    izquierda(minimal_publisher, 0.1, 90)
    recto(minimal_publisher, 0.1, 0.6)
    izquierda(minimal_publisher, 0.1, 90)
    recto(minimal_publisher, 0.1, 1.8)
    
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
