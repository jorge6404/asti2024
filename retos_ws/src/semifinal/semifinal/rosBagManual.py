import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from custom_interfaces.msg import SetVelocity
from geometry_msgs.msg import Twist
import time
import math
from semifinal.misfunciones import *


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

    
def save_inscript(datos, tipo_movimiento, fileName="datos_ros_playback.txt"):
    fichero = open(fileName, "a")
    cadena = f'{datos},{tipo_movimiento}\n'
    fichero.write(cadena)
    fichero.close()


def load_inscript(minimal_publisher, fileName="datos_ros_playback.txt"):
    fichero = open(fileName, "r")
    for linea in fichero:
        lista = linea.split(",")
        print(lista)
        print(linea)
        if lista[1] == "d\n":
            if float(lista[0]) > 0:
                recto(minimal_publisher, 0.1, float(lista[0]))
            else:
                atras(minimal_publisher, 0.1, abs(float(lista[0])))
        elif lista[1] == "g\n":
            if float(lista[0]) > 0:
                izquierda(minimal_publisher, 0.1, float(lista[0]))
            else:
                derecha(minimal_publisher, 0.1, abs(float(lista[0])))
        else:
            print("ERROR LECTURA FICHERO")
    fichero.close()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    executor = SingleThreadedExecutor()
    
    #Programa principal
    opcion = input("c para cargar, s para guardar: ")
    if opcion == "c":
        fileName = input("El nombre del fichero: ")
        load_inscript(minimal_publisher, fileName)
    elif opcion == "s":
        fileName = input("El nombre del fichero: ")
        mov_type = ""
        while mov_type != "x":
            mov_type = input(
                "Introduce movimiento (d para avanzar, g para girar): ")
            mov_data = float(input(
                "Introduce distancia en metros o giro en grados: "))

            if mov_type == "d":
                if mov_data > 0:
                    recto(minimal_publisher, 0.1, mov_data)
                else:
                    atras(minimal_publisher, 0.1, abs(mov_data))
            elif mov_type == "g":
                if mov_data > 0:
                    izquierda(minimal_publisher, 0.1, mov_data)
                else:
                    derecha(minimal_publisher, 0.1, abs(mov_data))
            elif mov_type == "x":
                break
            save_inscript(mov_data, mov_type, fileName)


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
