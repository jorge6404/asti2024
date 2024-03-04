import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16

"""
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

cont = 0

while True:

    success, img = cap.read()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # cv2.circle(img, (320, 480), 5, (50, 50, 255), 2)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([50, 50, 50])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    resultado = cv2.bitwise_and(img, img, mask=mask)

    hImg, wImg, _ = resultado.shape

    cuadricula = [
        [resultado[: hImg // 3, : wImg // 3], resultado[: hImg // 3, wImg // 3: 2 * wImg // 3],
         resultado[: hImg // 3, 2 * wImg // 3: wImg]],
        [resultado[hImg // 3: 2 * hImg // 3, : wImg // 3],
         resultado[hImg // 3: 2 * hImg // 3, wImg // 3: 2 * wImg // 3],
         resultado[hImg // 3: 2 * hImg // 3, 2 * wImg // 3: wImg]],
        [resultado[2 * hImg // 3: hImg:, : wImg // 3], resultado[2 * hImg // 3: hImg:, wImg // 3: 2 * wImg // 3],
         resultado[2 * hImg // 3: hImg:, 2 * wImg // 3: wImg]]
    ]
    
    cuadricula25 = [
        [resultado[i * hImg // 5:(i + 1) * hImg // 5, j * wImg // 5:(j + 1) * wImg // 5] for j in range(5)]
        for i in range(5)
    ]
    for i in range(5):
        for j in range(5):
            if np.any(cuadricula25[i][j] > 0):
                cv2.circle(img, ((j * 2 + 1) * wImg // 10, (i * 2 + 1) * hImg // 10), 5, (50, 50, 255), 2)
    
    cuadricula49 = [
        [resultado[i * hImg // 7:(i + 1) * hImg // 7, j * wImg // 7:(j + 1) * wImg // 7] for j in range(7)]
        for i in range(7)
    ]

    for i in range(7):
        for j in range(7):
            if np.any(cuadricula49[i][j] > 0):
                print(((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14))
                cv2.circle(img, ((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14), 5, (50, 50, 255), 2)

    cv2.imshow("Result", resultado)

    cv2.imshow("Video", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
"""


class DetectLinea(Node):
    def __init__(self):
        super().__init__('detectar_linea')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'linea', 10)
        self.publisher_
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)



    def timer_callback(self):
        self.detectar()

    def publish(self, tupla):
        msg = Int32MultiArray()
        msg.data = tupla
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def detectar(self):
        while True:
            success, img = self.cap.read()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.circle(img, (320, 480), 5, (50, 50, 255), 2)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([0, 0, 0])
            upper_blue = np.array([50, 50, 50])

            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            resultado = cv2.bitwise_and(img, img, mask=mask)

            hImg, wImg, _ = resultado.shape

            """
            cuadricula = [
                [resultado[: hImg // 3, : wImg // 3], resultado[: hImg // 3, wImg // 3: 2 * wImg // 3],
                 resultado[: hImg // 3, 2 * wImg // 3: wImg]],
                [resultado[hImg // 3: 2 * hImg // 3, : wImg // 3],
                 resultado[hImg // 3: 2 * hImg // 3, wImg // 3: 2 * wImg // 3],
                 resultado[hImg // 3: 2 * hImg // 3, 2 * wImg // 3: wImg]],
                [resultado[2 * hImg // 3: hImg:, : wImg // 3], resultado[2 * hImg // 3: hImg:, wImg // 3: 2 * wImg // 3],
                 resultado[2 * hImg // 3: hImg:, 2 * wImg // 3: wImg]]
            ]
            
            cuadricula25 = [
                [resultado[i * hImg // 5:(i + 1) * hImg // 5, j * wImg // 5:(j + 1) * wImg // 5] for j in range(5)]
                for i in range(5)
            ]
            for i in range(5):
                for j in range(5):
                    if np.any(cuadricula25[i][j] > 0):
                        cv2.circle(img, ((j * 2 + 1) * wImg // 10, (i * 2 + 1) * hImg // 10), 5, (50, 50, 255), 2)
            """

            cuadricula49 = [
                [resultado[i * hImg // 7:(i + 1) * hImg // 7, j * wImg // 7:(j + 1) * wImg // 7] for j in range(7)]
                for i in range(7)
            ]

            puntos = [[0]*7 for _ in range(7)]

            for i in range(7):
                for j in range(7):
                    if np.any(cuadricula49[i][j] > 0):
                        # print(((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14))
                        cv2.circle(img, ((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14), 5, (50, 50, 255), 2)
                        puntos[i][j] = 1

            suma_central, diferencia = self.calcular_diferencia_puntos(puntos)
            print(suma_central, diferencia, suma_central + diferencia)
            #self.publish((suma_columna_central, diferencia_posiciones))

            cv2.imshow("Result", resultado)
            cv2.imshow("Video", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def detectar_puntos_activos(self, resultado, hImg, wImg, puntos):
        for i in range(7):
            for j in range(7):
                if np.any(resultado[i * hImg // 7:(i + 1) * hImg // 7, j * wImg // 7:(j + 1) * wImg // 7] > 0):
                    puntos[i][j] = 1

    def calcular_diferencia_puntos(self, puntos):
        suma_columna_central = sum(puntos[i][3] for i in range(7))
        suma_derecha = sum(sum(puntos[i][4:]) for i in range(7))
        suma_izquierda = sum(sum(puntos[i][:3]) for i in range(7))
        diferencia_posiciones = suma_derecha - suma_izquierda
        return suma_columna_central, diferencia_posiciones


def main(args=None):
    rclpy.init(args=args)
    detectar_linea = DetectLinea()
    rclpy.spin(detectar_linea)
    detectar_linea.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()