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

def recto(pub, vel, tiempo):
    # Publish velocity to id 1 and 2
    pub.publish_velocity(1, -vel)
    pub.publish_velocity(2, vel)

    # Wait for 'tiempo' seconds
    time.sleep(tiempo)

    # Publish velocity 0 to id 1 and 2
    pub.publish_velocity(1, 0)
    pub.publish_velocity(2, 0)

def derecha(pub, vel, tiempo):
    # Publish velocity to id 1 and 2
    pub.publish_velocity(1, -vel)
    pub.publish_velocity(2, -vel)

    # Wait for 'tiempo' seconds
    time.sleep(tiempo)

    # Publish velocity 0 to id 1 and 2
    pub.publish_velocity(1, 0)
    pub.publish_velocity(2, 0)

def izquierda(pub, vel, tiempo):
    # Publish velocity to id 1 and 2
    pub.publish_velocity(1, vel)
    pub.publish_velocity(2, vel)

    # Wait for 'tiempo' seconds
    time.sleep(tiempo)

    # Publish velocity 0 to id 1 and 2
    pub.publish_velocity(1, 0)
    pub.publish_velocity(2, 0)

def atras(pub, vel, tiempo):
    # Publish velocity to id 1 and 2
    pub.publish_velocity(1, vel)
    pub.publish_velocity(2, -vel)

    # Wait for 'tiempo' seconds
    time.sleep(tiempo)

    # Publish velocity 0 to id 1 and 2
    pub.publish_velocity(1, 0)
    pub.publish_velocity(2, 0)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()

    time.sleep(2)
    recto(minimal_publisher, 200, 4)
    derecha(minimal_publisher, 180, 2)
    recto(minimal_publisher, 200, 4)


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