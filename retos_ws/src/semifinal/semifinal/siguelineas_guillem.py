import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from semifinal.misfunciones import *


class DetectLinea(Node):
    def __init__(self):
        super().__init__('detectar_linea')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(2)  # /home/jcrex/Vídeos/siguelineas_largo.mp4
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.divisiones = 7

        self.contador = 0.01
        self.memoria = 0

    def timer_callback(self):
        self.detectar()

    def publish(self, tupla):
        msg = Int32MultiArray()
        msg.data = tupla
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def publish_velocity(self, velocity):            # velocity -->  tuple = (1, 0) (linear.x, angular.z)
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.angular.z = velocity[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

    def camara(self):
        # Uso para el testeo de camara
        while True:
            success, img = self.cap.read()
            cv2.imshow("Video", img)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break

    def detectar_puntos_activos(self, cuadricula, hImg, wImg, img, puntos):
        for i in range(self.divisiones):
            for j in range(self.divisiones):
                if np.any(cuadricula[i][j] > 0):
                    # print(((j * 2 + 1) * wImg // 14, (i * 2 + 1) * hImg // 14))
                    cv2.circle(img, ((j * 2 + 1) * wImg // (2*self.divisiones), (i * 2 + 1) * hImg // (2*self.divisiones)),
                               5, (50, 50, 255), 2)
                    if (i > (self.divisiones - 3) and j < 2) or (i < 2 and j < 2):
                        puntos[i][j] = 2
                    elif (i > (self.divisiones - 3) and j > (self.divisiones - 3)) or (i < 2 and j > (self.divisiones - 3)):
                        puntos[i][j] = 2
                    else:
                        puntos[i][j] = 1

    def calcular_diferencia_puntos(self, puntos):
        suma_columna_central = sum(puntos[i][self.divisiones // 2] for i in range(self.divisiones))
        suma_derecha = sum(sum(puntos[i][self.divisiones // 2 + 1:]) for i in range(self.divisiones))
        suma_izquierda = sum(sum(puntos[i][:self.divisiones // 2 - 1]) for i in range(self.divisiones))
        diferencia_posiciones = suma_derecha - suma_izquierda
        #print(suma_derecha, suma_izquierda, diferencia_posiciones)
        return suma_columna_central, diferencia_posiciones

    def movimiento(self, vel):
        if vel < -self.divisiones + (self.divisiones // 2):
            if self.contador > 0:
                print("Girar izquierda 2")
                self.publish_velocity((0.0, 0.1 - self.contador))
                self.contador -= 0.01
            else:
                print("Girar izquierda 1")
                self.publish_velocity((0.0, 0.1))
            self.memoria = -1
            print(self.memoria)

        elif vel > self.divisiones // 0.9:
            if self.contador > 0:
                print("Girar derecha 2")
                self.publish_velocity((0.0, -0.1 + self.contador))
                self.contador += 0.01
            else:
                print("Girar derecha 1")
                self.publish_velocity((0.0, -0.1))
            self.memoria = 1
            print(self.memoria)
        else:
            print("ir recto 0")
            self.publish_velocity((0.1, 0.0))
            self.contador = 0.0
            #self.memoria = 0
            print(self.memoria)

    def detectar(self):
        while True:
            success, img = self.cap.read()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([0, 0, 0])
            upper_blue = np.array([35, 35, 35])

            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            resultado = cv2.bitwise_and(img, img, mask=mask)

            hImg, wImg, _ = resultado.shape

            cuadricula = [
                [resultado[i * hImg // self.divisiones:(i + 1) * hImg // self.divisiones,
                 j * wImg // self.divisiones:(j + 1) * wImg // self.divisiones] for j in range(self.divisiones)]
                for i in range(self.divisiones)
            ]

            puntos = [[0] * self.divisiones for _ in range(self.divisiones)]
            self.detectar_puntos_activos(cuadricula, hImg, wImg, img, puntos)

            suma_central, diferencia = self.calcular_diferencia_puntos(puntos)
            #print(suma_central, diferencia, suma_central + diferencia, self.divisiones + diferencia)

            """
            if suma_central == diferencia == 0:
                if self.contador > 0.0:
                    self.publish_velocity((0.0, 0.1))
                self.contador += 0.01
            elif suma_central == diferencia == 0:
                if self.contador > 2.0:
                    self.publish_velocity((0.0, -0.1))
                self.contador -= 0.01
            else:
                self.movimiento(diferencia)
            
            """

            if suma_central == diferencia == 0:
                if self.memoria == 0:
                    print("memoria recta")
                    self.publish_velocity((-0.1, 0.0))
                elif self.memoria == 1:
                    self.publish_velocity((0.0, -0.1))
                elif self.memoria == -1:
                    self.publish_velocity((0.0, 0.1))

            else:
                self.movimiento(diferencia)

            #self.publish((suma_central, diferencia))

            cv2.imshow("Result", resultado)
            cv2.imshow("Video", img)

            if cv2.waitKey(100) & 0xFF == ord('q'):
                break


def menu():

    opcion = int(input(
        "Menu de control del siguelineas elegir: \n"
        "1 para empezar a detectar linea y moverse \n"
        "2 para controlar los movimiento de forma manual \n"
        "3 Mostrar la camara y luego ejecutar codigo \n"
    ))
    print(f"Opcion seleccionada {opcion}")
    return opcion


def main(args=None):
    opcion = menu()
    if opcion == 1:
        rclpy.init(args=args)
        detectar_linea = DetectLinea()
        rclpy.spin(detectar_linea)
        detectar_linea.destroy_node()

    elif opcion == 2:
        rclpy.init(args=args)
        movimiento = DetectLinea()
        opcion = input(
            "Elija la direccion que tomara: \n"
            "Valor < -3 girar a la izquierda \n"
            "Valor > 3 girar a la derecha \n"
            "Valor intermedio ir recto \n"
            "No poner nada acaba el programa \n"
        )
        while opcion != "":
            movimiento.movimiento(int(opcion))
            opcion = input(
                "Elija la direccion que tomara: \n"
                "Valor < -3 girar a la izquierda \n"
                "Valor > 3 girar a la derecha \n"
                "Valor intermedio ir recto \n"
                "No poner nada acaba el programa \n"
            )
            movimiento.publish_velocity((0.0, 0.0))
        #rclpy.spin(movimiento)
        movimiento.destroy_node()

    elif opcion == 3:
        rclpy.init(args=args)
        camara = DetectLinea()
        print("activada")
        camara.camara()
        rclpy.spin(camara)
        camara.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()