import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from semifinal.misfunciones import *

class DetectLinea(Node):
    def __init__(self):
        super().__init__('detectar_linea')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def timer_callback(self):
        self.detectar()

    def publish_velocity(self, velocity):            # velocity -->  tuple = (1, 0) (linear.x, angular.z)
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.angular.z = velocity[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

    def detectar(self):
        while True:
            success, img = self.cap.read()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.circle(img, (320, 480), 5, (50, 50, 255), 2)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([0, 0, 0])
            upper_blue = np.array([80, 80, 80])

            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            resultado = cv2.bitwise_and(img, img, mask=mask)

            hImg, wImg, _ = resultado.shape

            cuadricula49 = [
                [resultado[i * hImg // 7:(i + 1) * hImg // 7, j * wImg // 7:(j + 1) * wImg // 7] for j in range(7)]
                for i in range(7)
            ]

            puntos = [[0]*7 for _ in range(7)]
            self.detectar_puntos_activos(resultado, hImg, wImg, img, puntos)
            
            diferencia = self.calcular_diferencia_puntos(puntos)

            #self.publish((suma_columna_central, diferencia_posiciones))
            ####
            # cv2.imshow("Result", resultado)
            cv2.imshow("Video", img)
            ###

            """
            ESTO CORRESPONDE AL MAIN():
            """
            if diferencia < -5:
                print("girar izquierda")
                girar_izquierda(self)

            elif diferencia > 5:
                print("girar derecha")
                girar_derecha(self)

            else:
                print("avanzar recto")
                avanzar(self)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def detectar_puntos_activos(self, resultado, hImg, wImg, img, puntos):
        for i in range(7):
            for j in range(7):
                if np.any(resultado[i * hImg // 7:(i + 1) * hImg // 7, j * wImg // 7:(j + 1) * wImg // 7] > 0):
                    cv2.circle(img, ((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14), 5, (50, 50, 255), 2)
                    puntos[i][j] = 1
                    
    
    def calcular_diferencia_puntos(self, puntos):
        suma_derecha = sum(sum(puntos[i][4:]) for i in range(7))
        suma_izquierda = sum(sum(puntos[i][:3]) for i in range(7))
        diferencia_posiciones = suma_derecha - suma_izquierda
        print(suma_izquierda, suma_derecha, diferencia_posiciones)
        return diferencia_posiciones


def main(args=None):
    rclpy.init(args=args)
    detectar_linea = DetectLinea()
    detectar_linea.detectar()
    rclpy.spin(detectar_linea)
    detectar_linea.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
