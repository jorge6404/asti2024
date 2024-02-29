import rclpy
from std_msgs.msg import String

class MyNode(rclpy.Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  