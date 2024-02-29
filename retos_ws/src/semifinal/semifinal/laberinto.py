import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SetVelocity

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SetVelocity, 'set_velocity', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = SetVelocity()
        msg.id = 1
        msg.velocity = 5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.id)
        self.get_logger().info('Publishing: "%s"' % msg.velocity)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()