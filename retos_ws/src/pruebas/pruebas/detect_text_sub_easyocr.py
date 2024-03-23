import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from easyocr import Reader
from pytesseract import Output
import imutils


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

        self.caracteres = [input("Ingrese el caracter numero: "), input("Ingrese el caracter letra: ")]

        self.primer = True

    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)
        # Aplicamos OCR utilizando los lenguajes definidos anteriormente.
        reader = Reader(['en', 'es'], gpu=True)
        results = reader.readtext(img)

        # Iteramos sobre las predicciones del modelo de EasyOCR.
        for bounding_box, text, probability in results:
            # Imprimimos la probabilidad del texto.
            print(f'{probability:.4f}: {text}')

            # Extraemos y ajustamos las coordenadas de la detección.
            tl, tr, br, bl = bounding_box
            tl = (int(tl[0]), int(tl[1]))
            tr = (int(tr[0]), int(tr[1]))
            br = (int(br[0]), int(br[1]))
            bl = (int(bl[0]), int(bl[1]))

            # Limpiamos el texto, y lo mostramos en la imagen.
            text = cleanup_text(text)
            cv2.rectangle(img, tl, br, (0, 255, 0), 2)
            cv2.putText(img, text, (tl[0], tl[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)

        # Mostramos el resultado en pantalla.
        cv2.imshow('Resultado', img)
        cv2.waitKey(1)


def cleanup_text(text):
    return ''.join([c if ord(c) < 128 else '' for c in text]).strip()


def pintar(x, y, w, h, img, b, hImg, wImg):
    #print(x, y, w, h, b, hImg, wImg)
    cv2.rectangle(img, (x, hImg - y), (w, hImg - h), (50, 50, 255), 2)
    cv2.line(img, (wImg//2, hImg), (x, hImg - y), (50, 50, 255), 2)
    cv2.putText(img, b[0], (x, hImg - y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 255), 2)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()