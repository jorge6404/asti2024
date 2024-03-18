import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from Model_detect_text import CamAstic

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        """
        self.subscription_array = self.create_subscription(
            Int32MultiArray,
            'array_topic',
            self.listener_callback_array,
            10)
        self.subscription_array
        """

        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)

        asti = CamAstic()

        """ El subcritor lo que hara sera agarrar las coordenadas dadas por el publisher para mover
            El motor a la dirreccion que queremos."""

        cv2.imshow("camera", img)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()