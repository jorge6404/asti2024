import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from custom_interfaces.msg import SetPosition

'''
from .Movements import Movements
mov = Movements()

# Queremos probar si el robot hace todos los movimientos correctamente?:
mov.prueba_movimientos()

#Queremos ejecutar movimientos básicos?:
mov.avanzar()
mov.retroceder()
mov.detener()
mov.girar_izquierda()
mov.girar_derecha()

# Queremos ejecutar más movimientos avanzados, en base a distancias, radios, etc?:
mov.avanzar_distancia(distancia_total=1)
girar_grados(self, degrees=90, direccion=izq, radio=1)

# Va muy rápido el robot para la prueba concreta, y queremos una aceleración más progresiva para cualquier movimiento?
mov.actualizar_velocidades(self, max_linear_vel=0.1, max_angular_vel=1, linear_acc=0.01, angular_acc=0.1):

POR QUÉ ES ÚTIL?
Lo interesante de esto, es el hecho de poder simplificar mucho más el código, y no tener que estar preocupándonos por las velocidades, aceleraciones...  
Esto ya estaba más o menos implementado antes, pero ahora está mejor organizado y más fácil de usar.

Ejemplo:
'''

class Movements(Node):
    def __init__(self):
        """
        Clase Personalizada para el manejo de movimientos del robot
        
        EJEMPLO TOOLS:
        from .Movements import Movements      (Falta ver si este tipo de importación en la raspberry funciona bien)
        mov = Movements()
        mov.herramienta_girar(90)
        mov.boli_subir()
        mov.boli_bajar()
        mov.bolos_soltar()
        mov.bolos_mantener()
        mov.pale_subir()
        mov.pale_bajar()
        """
        
        super().__init__('movement_publisher')
        self.wheel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)   # cmd_vel has (m/s , rad/s)
        self.tool_publisher_ = self.create_publisher(SetPosition, 'tool_pos', 10)
        
        # WHEELS
        self.max_linear_vel = 0.3       # TODO: CAMBIAR A VALORES QUE SEAN BUENOS POR DEFECTO
        self.max_angular_vel = 2.0
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        self.last_vel = (0.0, 0.0)
        
        # TOOL
        self.tool_pos = 0.0
        self.tool_id = 3
        self.grados_boli_alto = 0.0     # TODO: Cambiar a valor correcto para cada prueba
        self.grados_boli_bajo = 0.0     
        self.grados_bolos_soltar = 0.0  
        self.grados_bolos_mantener = 0.0
        self.grados_pale_alto = 0.0     
        self.grados_pale_bajo = 0.0     


    # WHEELS
    
    def publish_wheel_velocity(self, vel_x, vel_y):
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_y
        self.wheel_publisher_.publish(msg)
        
        self.last_vel = (vel_x, vel_y)
        self.show_current_vel()

    def show_current_vel(self):
        self.get_logger().info(f'Current velocity: {self.last_vel}')
        
    def actualizar_velocidades(self, max_linear_vel, max_angular_vel, linear_acc, angular_acc):
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.linear_acc = linear_acc
        self.angular_acc = angular_acc
        
    # ╔═════════════════════════════╗
    # ║       TOOL FUNCTIONS        ║
    # ╚═════════════════════════════╝
    
    def herramienta_girar(self, grados):
        msg = SetPosition()
        msg.position = grados
        msg.id = self.tool_id
        self.tool_publisher_.publish(msg)
        
    # Dibujar la figura
    
    def boli_subir(self):
        self.herramienta_girar(self.grados_boli_alto)
        
    def boli_bajar(self):
        self.herramienta_girar(self.grados_boli_bajo)
        
    # Bolos
    
    def bolos_soltar(self):
        self.herramienta_girar(self.grados_bolos_soltar)
        
    def bolos_mantener(self):
        self.herramienta_girar(self.grados_bolos_mantener)
        
    # Mini-fábrica
    
    def pale_subir(self):
        self.herramienta_girar(self.grados_pale_alto)
        
    def pale_bajar(self):
        self.herramienta_girar(self.grados_pale_bajo)
        

    # ╔═════════════════════════════╗
    # ║ MOVIMIENTOS BÁSICOS RUEDAS  ║
    # ╚═════════════════════════════╝
    
    def avanzar(self):
        self.publish_wheel_velocity(self.max_linear_vel, 0.0)
        
    def retroceder(self):
        self.publish_wheel_velocity(-self.max_linear_vel, 0.0)
        
    def girar_izquierda(self):
        self.publish_wheel_velocity(0.0, self.max_angular_vel)
        
    def girar_derecha(self):
        self.publish_wheel_velocity(0.0, -self.max_angular_vel)
        
    def detener(self):
        self.publish_wheel_velocity(0.0, 0.0)
        

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
            self.publish_wheel_velocity(vel_linear, 0.0)
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
            self.publish_wheel_velocity(-vel_linear, 0.0)
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
        
        # --- CON ACELERACIÓN ---      # TODO: No funcionaba bien
        # if radio == 0.0:
        #     while(radian_total >= 0):
        #         if (vel_ang < max_ang):
        #             vel_ang += acc_ang
        #         self.publish_wheel_velocity(0.0, vel_ang*direccion)
        #         time.sleep(0.1)
        #         grados_recorridos = vel_ang*0.1
        #         radian_total -= grados_recorridos
        #     self.detener()
        
        # --- SIN ACELERACIÓN ---
        if radio == 0.0:
            while(radian_total >= 0):
                self.publish_wheel_velocity(0.0, max_ang*direccion)
                time.sleep(0.1)
                grados_recorridos = max_ang*0.1
                radian_total -= grados_recorridos
            self.detener()


        # GIRO CON RADIO (velocidad lineal != 0.0)
        else:
            # TODO: Arreglar esto
            
            print("Giro con radio")
            angular_vel = float(vel_lin/radian_total*10)        # ???
            
            while(radian_total >= 0.0):
                print("Angular velocity: ", angular_vel)
                
                self.publish_wheel_velocity(vel_lin, angular_vel*direccion)
                
                time.sleep(0.1)
                radian_recorridos = angular_vel * 0.1
                radian_total -= radian_recorridos

            self.detener()

    def girar_grados_izq(self, degrees, radio=0.0):
        self.girar_grados(degrees, 'izq', radio)
        
    def girar_grados_der(self, degrees, radio=0.0):
        self.girar_grados(degrees, 'der', radio)