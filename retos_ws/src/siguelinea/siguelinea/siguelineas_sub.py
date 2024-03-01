import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from custom_interfaces.msg import SetVelocity


class Linea_sub(Node):

    def __init__(self):
        super().__init__('linea_sub')
        #self.publisher_ = self.create_publisher(Int16, 'linea', 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'linea', self.listener_callback, 10)
        self.subscription

        self.publisher_ = self.create_publisher(
            SetVelocity,
            'set_velocity',
            10)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        print(msg.data[0], msg.data[1])
        self.get_logger().info(f"Publicando: {msg.data}")
        self.publisher_.publish(msg)



def main(args=None):

    rclpy.init(args=args)
    linea_sub = Linea_sub()
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
