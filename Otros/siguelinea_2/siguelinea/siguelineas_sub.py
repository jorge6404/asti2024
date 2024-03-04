import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
#from custom_interfaces.msg import SetVelocity
from geometry_msgs.msg import Twist

from semifinal.misfunciones import *


class Linea_sub(Node):

    def __init__(self):
        super().__init__('linea_sub')
        #self.publisher_ = self.create_publisher(Int16, 'linea', 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'linea', self.listener_callback, 10)
        self.subscription

        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

    def publish_velocity(self, velocity):            # velocity -->  tuple = (1, 0) (linear.x, angular.z)
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.angular.z = velocity[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        print(msg.data, type(msg.data), msg.data[0], msg.data[1], type(msg.data[0]), type(msg.data[1]))
        self.get_logger().info(f"Publicando: {msg.data}")
        #self.publisher_.publish(msg)

        #cmd_vel = Twist()
        self.movimiento(msg)
        #self.publisher_.publish(cmd_vel)
        #self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

    def movimiento(self, msg):
        distancia = 1.0
        velocidad = 0.2
        print(msg.data[0], msg.data[1])
        if msg.data[0] == msg.data[1] != 0:
            print('Recto')
            #recto(self, velocidad, distancia)
            self.publish_velocity((velocidad, 0.1))
        elif msg.data[0] > msg.data[1]:
            print('Derecha')
            #derecha(self, velocidad, distancia)
            self.publish_velocity((0.0, -velocidad))
        elif msg.data[1] > msg.data[0] > 0:
            print('Izquierda')
            #izquierda(self, velocidad, distancia)
            self.publish_velocity((0.0, velocidad))
        elif msg.data[0] < 0:
            print('Giro 180º')
            #izquierda(self, velocidad, 2*distancia)
            self.publish_velocity((0.0, -velocidad))
        elif msg.data[0] == msg.data[1] == 0:
            print('Detenido')
            #recto(self, 0, 0)
            self.publish_velocity((0.0, 0.0))


def main(args=None):

    rclpy.init(args=args)
    linea_sub = Linea_sub()
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()