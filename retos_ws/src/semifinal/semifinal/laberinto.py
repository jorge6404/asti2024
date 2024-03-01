import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
from geometry_msgs.msg import Twist
import time
import math

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

def recto(pub, max_vel, distancia_total):
    
    vel_linear = 0.01
    while(distancia_total > 0):
        if (vel_linear < max_vel):
            vel_linear += 0.005
        pub.publish_velocity((vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))


def atras(pub, max_vel, grados_totales):
    
    vel_angular = 0.01
    while(grados_totales > 0):
        if (vel_angular < max_vel):
            vel_angular += 0.01
        pub.publish_velocity((0.0, vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_totales -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))

def derecha(pub, max_vel, degrees):
    grados_total = degrees*math.pi/180
    
    vel_angular = 0.01
    while(grados_total > 0):
        if (vel_angular < max_vel):
            vel_angular += 0.01
        pub.publish_velocity((0.0, -vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_total -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))

def izquierda(pub, max_vel, grados_total):
    
    vel_linear = 0.01
    while(distancia_total > 0):
        if (vel_linear < max_vel):
            vel_linear += 0.01
        pub.publish_velocity((-vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()

    time.sleep(2)
    recto(minimal_publisher, 0.1, 1)
    derecha(minimal_publisher, 0.1, 90)


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
