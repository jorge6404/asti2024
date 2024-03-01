import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
import time

class TranslateNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(SetVelocity, 'set_velocity', 10)
        self.subscription = self.create_subscription(
            SetVelocity,
            'pure_velocity',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, id,  msg):
        msg = SetVelocity()
        msg.id = id
        msg.velocity = round(msg.velocity * 4.3668)  # Redondea al entero más cercano
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: id="{msg.id}", velocity="{msg.velocity}"')

def main(args=None):
    rclpy.init(args=args)

    node = TranslateNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()