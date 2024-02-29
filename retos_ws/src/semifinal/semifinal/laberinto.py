import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SetVelocity, 'set_velocity', 10)

    def publish_velocity(self, id, velocity):
        msg = SetVelocity()
        msg.id = id
        msg.velocity = velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: id="{msg.id}", velocity="{msg.velocity}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()

    # Publish velocity to id 1 and 2
    minimal_publisher.publish_velocity(1, 20)
    minimal_publisher.publish_velocity(2, -20)

    # Wait for 3 seconds
    time.sleep(3)

    # Publish velocity 0 to id 1 and 2
    minimal_publisher.publish_velocity(1, 0)
    minimal_publisher.publish_velocity(2, 0)

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