import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

'''
Lo interesante de esto, es el hecho de poder simplificar mucho más el código, y no tener que estar preocupándonos por las velocidades, aceleraciones...  
Esto ya estaba más o menos implementado antes, pero ahora está mejor organizado y más fácil de usar.

Ejemplo:

from .Movements import Movements
mov = Movements()

# Queremos probar si el robot hace todos los movimientos correctamente?:
mov.prueba_movimientos()

#Queremos ejecutar movimientos básicos?:
mov.avanzar()
mov.detener()
mov.girar_izquierda()
mov.girar_derecha()

# Queremos ejecutar más movimientos avanzados, en base a distancias, radios, etc?:
mov.avanzar_distancia(distancia_total=1)
girar_grados(self, degrees=90, direccion=izq, radio=1)

# Va muy rápido el robot para la prueba concreta, y queremos una aceleración más progresiva para cualquier movimiento?
mov.actualizar_velocidades(self, max_linear_vel=0.1, max_angular_vel=1, linear_acc=0.01, angular_acc=0.1):
'''

class Movements(Node):
    def __init__(self):
        """
        Clase Personalizada para el manejo de movimientos del robot
        
        Funciones:
        - avanzar()
        - retroceder
        - girar_izquierda
        - girar_derecha
        - detener
        - avanzar_distancia
        - retroceder_distancia
        - girar_grados
        - prueba_movimientos
        """
        
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)   # cmd_vel has (m/s , rad/s)
        self.max_linear_vel = 0.3       # TODO: CAMBIAR A VALORES QUE SEAN BUENOS POR DEFECTO
        self.max_angular_vel = 2.0
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        
        self.last_vel = (0.0, 0.0)

    def publish_velocity(self, vel_x, vel_y):
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_y
        self.publisher_.publish(msg)
        
        self.last_vel = (vel_x, vel_y)
        self.show_current_vel()

    def show_current_vel(self):
        self.get_logger().info(f'Current velocity: {self.last_vel}')
        
    def actualizar_velocidades(self, max_linear_vel, max_angular_vel, linear_acc, angular_acc):
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.linear_acc = linear_acc
        self.angular_acc = angular_acc
    
    # ╔═════════════════════╗
    # ║ MOVIMIENTOS BÁSICOS ║
    # ╚═════════════════════╝
    
    def avanzar(self):
        self.publish_velocity(self.max_linear_vel, 0.0)
        
    def retroceder(self):
        self.publish_velocity(-self.max_linear_vel, 0.0)
        
    def girar_izquierda(self):
        self.publish_velocity(0.0, self.max_angular_vel)
        
    def girar_derecha(self):
        self.publish_velocity(0.0, -self.max_angular_vel)
        
    def detener(self):
        self.publish_velocity(0.0, 0.0)

    # ╔═══════════════════════╗
    # ║ PRUEBA DE MOVIMIENTOS ║
    # ╚═══════════════════════╝
    
    def prueba_movimientos(self):
        print("Realizando prueba de movimiento, espere 3 segundos...")
        
        # Move forward
        self.avanzar()
        time.sleep(1.0)
        
        # Move backward
        self.retroceder()
        time.sleep(1.0)
        
        # Turn left
        self.girar_derecha()
        time.sleep(1.0)
        
        # Turn right
        self.girar_izquierda()
        time.sleep(1.0)
        
        # Stop
        self.detener()

    # ╔═══════════════════════════════════════════╗
    # ║ FUNCIONES POR DISTANCIAS, RADIOS O GRADOS ║
    # ╚═══════════════════════════════════════════╝
    
    def avanzar_distancia(self, distancia_total):
        # Con aceleración
        vel_linear = 0.0
        while(distancia_total >= 0):
            if (vel_linear < self.max_linear_vel):
                vel_linear += self.linear_acc
            self.publish_velocity(vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia_total -= distancia_recorrida
        self.detener()

    def retroceder_distancia(self, distancia_total):
        # Con aceleración
        vel_linear = 0.0
        while(distancia_total >= 0):
            if (vel_linear < self.max_linear_vel):
                vel_linear += self.linear_acc
            self.publish_velocity(-vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia_total -= distancia_recorrida
        self.detener()


    def girar_grados(self, degrees, direccion, radio=0.0):
        # TODO: Testear que está bien, ya que no lo he probado
        
        """Rotar el robot en un angulo determinado

        Args:
            degrees (int): Grados a rotar (en decimal) (Siempre se convertirá en positivo)
            direccion (str): 'izq' o 'der'
            radio (float, optional): Si se quiere realizar el movimiento con un radio. Defaults to 0.0.
            config (dict, optional): Configuración adicional sobre velocidades. Defaults to {'max_linear': 0.3, 'max_angular': 2, 'acc_linear': 0.01, 'acc_angular': 0.1}.
        """
        
        # DATOS
        radian_total = abs(degrees)*math.pi/180
        if direccion == 'izq':
            direccion = 1         # todo: Está bien?
        else:
            direccion = -1
        
        vel_lin = self.max_linear_vel
        vel_ang = 0.0

        max_ang = self.max_angular_vel
        acc_ang = self.angular_acc


        # GIRO SOBRE SÍ MISMO (velocidad lineal = 0.0)
        if radio == 0.0:
            while(radian_total >= 0):
                if (vel_ang < max_ang):
                    vel_ang += acc_ang
                self.publish_velocity(0.0, vel_ang*direccion)
                time.sleep(0.1)
                grados_recorridos = vel_ang*0.1
                radian_total -= grados_recorridos

            self.detener()


        # GIRO CON RADIO (velocidad lineal != 0.0)
        else:
            while(radian_total >= 0.2):          # TODO: ver vien
                angular_vel:float = vel_lin/radian_total
                
                self.publish_velocity(vel_lin, -angular_vel)
                
                time.sleep(0.1)
                radian_recorridos = angular_vel * 0.1
                radian_total -= radian_recorridos

            self.detener()
