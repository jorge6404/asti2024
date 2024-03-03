import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16


class LineaPublisher(Node):

    def __init__(self):
        super().__init__('linea_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'linea', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tupla = (0, 0)
        self.matrix = np.zeros((3, 3), dtype=int)
        self.black_threshold = 50
        self.vid = cv2.VideoCapture(0)

        self.estacionado = True
        self.giro = "izq"
        self.counter = 300

    def timer_callback(self):

        self.run()

    def check_for_black(self, region):
        """
        frame = cv2.cvtColor(region, cv2.COLOR_BGR2RGB)
        # cv2.circle(frame, (320, 480), 5, (50, 50, 255), 2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 0, 0])
        upper_blue = np.array([100, 100, 100])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        """
        return np.any(region < self.black_threshold)

    def run(self):
        while True:
            try:
                ret, frame = self.vid.read()
                rows, cols, _ = frame.shape
            except AttributeError:
                break

            cell_size_x = cols // 3
            cell_size_y = rows // 3
            for i in range(3):
                for j in range(3):
                    roi = frame[i * cell_size_y:(i + 1) * cell_size_y, j * cell_size_x:(j + 1) * cell_size_x]
                    if self.check_for_black(roi):
                        self.matrix[i, j] = 1
                    else:
                        self.matrix[i, j] = 0

            cv2.imshow('frame', frame)

            if self.matrix[0, 2] == 1 and self.matrix[0, 0] == 0:
                self.estacionado = False
                self.tupla = (75, 50)
            elif self.matrix[0, 0] == 1 and self.matrix[0, 2] == 0:
                self.estacionado = False
                self.tupla = (50, 75)
            elif self.matrix[0, 1] == 1 and self.matrix[0, 0] == 0 and self.matrix[0, 2] == 0:
                self.estacionado = False
                self.tupla = (50, 50)
            elif self.matrix[0, 1] == 0 and self.matrix[0, 0] == 0 and self.matrix[0, 2] == 0 and self.matrix[
                1, 1] == 1 and \
                    self.matrix[2, 1] == 1:
                self.estacionado = False
                if self.giro == 'izq':
                    self.tupla = (-50, 50)
                elif self.giro == 'der':
                    self.tupla = (50, -50)
            elif self.matrix[0, 0] == 1 and self.matrix[0, 2] == 1 and self.matrix[2, 0] == 1 and self.matrix[
                2, 2] == 0:
                self.estacionado = False
                self.giro = 'izq'
            elif self.matrix[0, 0] == 1 and self.matrix[0, 2] == 1 and self.matrix[2, 0] == 0 and self.matrix[
                2, 2] == 1:
                self.estacionado = False
                self.giro = 'der'
            else:
                todos_cero = np.all(self.matrix == 0)
                if todos_cero and self.estacionado is False:
                    self.counter -= 1
                    print(f'Counter: {self.counter}')
                    if self.counter == 0:
                        self.estacionado = True
                    if todos_cero and self.estacionado:
                        self.tupla = (0, 0)
                        self.counter = 300

            # Print tuple and control signals
            if self.tupla[0] == self.tupla[1] != 0:
                print('Recto')
            elif self.tupla[0] > self.tupla[1]:
                print('Derecha')
            elif self.tupla[1] > self.tupla[0] > 0:
                print('Izquierda')
            elif self.tupla[0] < 0:
                print('Giro 180º')
            elif self.tupla[0] == self.tupla[1] == 0 and self.estacionado:
                print('Detenido')

            # Publish tuple data
            msg = Int32MultiArray()
            msg.data = list(self.tupla)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publicando: {msg.data}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.vid.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    linea_pub = LineaPublisher()
    rclpy.spin(linea_pub)
    linea_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
