import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from custom_interfaces.msg import SetPosition

def get_movements():
    # TODO: Importante mantener actualizado
    actions = {
        1: ('avanzar', []),
        2: ('retroceder', []),
        3: ('girar_izquierda', []),
        4: ('girar_derecha', []),
        5: ('detener', []),
        6: ('avanzar_distancia', ['distancia', 'aceleracion']),
        7: ('retroceder_distancia', ['distancia', 'aceleracion']),
        8: ('girar_grados_izq', ['degrees', 'radio']),
        9: ('girar_grados_der', ['degrees', 'radio']),
        10: ('girar_grados_izq_atras', ['degrees', 'radio']),
        11: ('girar_grados_der_atras', ['degrees', 'radio']),
        12: ('herramienta_girar', ['grados']),
        13: ('boli_subir', []),
        14: ('boli_bajar', []),
        15: ('bolos_soltar', []),
        16: ('bolos_mantener', []),
        17: ('pale_subir', []),
        18: ('pale_bajar', []),
        19: ('actualizar_vel_lineal', ['max_linear_vel']),
        20: ('actualizar_vel_angular', ['max_angular_vel']),
        21: ('actualizar_acc_lineal', ['linear_acc']),
        22: ('actualizar_acc_angular', ['angular_acc']),
        23: ('prueba_movimientos', []),
    }
    return actions

class Movements(Node):
    def __init__(self):
        """
        Clase Personalizada para el manejo de movimientos del robot.
        
        SETUP INICIAL:
        from .Movements import Movements      (Falta ver si este tipo de importación en la raspberry funciona bien)
        mov = Movements()
        
        EJEMPLO RUEDAS BÁSICOS:
        mov.avanzar()
        mov.retroceder()
        mov.girar_izquierda()
        mov.girar_derecha()
        mov.detener()
        mov.prueba_movimientos()      (Avanzar, retroceder, girar izquierda, girar derecha, detenerse -> Para probar que todo funciona bien)
        
        EJEMPLO RUEDAS POR DISTANCIAS, RADIOS, GRADOS:
        mov.avanzar_distancia(1)        (Avanzar 1 metro)
        mov.retroceder_distancia(1)
        mov.girar_grados_izq(90)        (Radio 0.0 por defecto)
        mov.girar_grados_der(75)
        mov.girar_grados_izq(90, radio=1.5)
        
        EJEMPLO MODIFICAR VELOCIDADES:
        mov.actualizar_vel_lineal(0.1)
        mov.actualizar_vel_angular(1)
        mov.actualizar_acc_lineal(0.01)
        mov.actualizar_acc_angular(0.1)
        
        EJEMPLO TOOLS:
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
        self.max_linear_vel = 0.1       # TODO: CAMBIAR A VALORES QUE SEAN BUENOS POR DEFECTO
        self.max_angular_vel = 0.7
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        self.last_vel = (0.0, 0.0)
        
        # TOOL
        self.tool_pos = 0.0
        self.tool_id = 3    # id 1,2 ruedas, id 3 herramienta
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
        # self.show_current_vel()       # Comentar si no se quiere mostrar la velocidad actual

    def show_current_vel(self):
        self.get_logger().info(f'Current velocity: {self.last_vel}')
        
    def actualizar_vel_lineal(self, max_linear_vel):
        self.max_linear_vel = max_linear_vel
    
    def actualizar_vel_angular(self, max_angular_vel):
        self.max_angular_vel = max_angular_vel
    
    def actualizar_acc_lineal(self, linear_acc):
        self.linear_acc = linear_acc
    
    def actualizar_acc_angular(self, angular_acc):
        self.angular_acc = angular_acc
    
    # ╔═════════════════════════════╗
    # ║       TOOL FUNCTIONS        ║
    # ╚═════════════════════════════╝
    
    def herramienta_girar(self, grados):
        msg = SetPosition()
        msg.position = int(grados)
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
    
    def avanzar_distancia(self, distancia, aceleracion=True):
        # Con aceleración
        vel_linear = 0.0
        while(distancia >= 0):
            if aceleracion:
                if (vel_linear < self.max_linear_vel):
                    vel_linear += self.linear_acc
            else:
                vel_linear = self.max_linear_vel
            self.publish_wheel_velocity(vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia -= distancia_recorrida
        self.detener()

    def retroceder_distancia(self, distancia, aceleracion=True):
        # Con aceleración
        vel_linear = 0.0
        while(distancia >= 0):
            if aceleracion:
                if (vel_linear < self.max_linear_vel):
                    vel_linear += self.linear_acc
            else:
                vel_linear = self.max_linear_vel
            self.publish_wheel_velocity(-vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia -= distancia_recorrida
        self.detener()


    def girar_grados_izq(self, degrees, radio=0.0):
        self.girar_grados(degrees, 'izq', radio)
        
    def girar_grados_der(self, degrees, radio=0.0):
        self.girar_grados(degrees, 'der', radio)
        
    def girar_grados_izq_atras(self, degrees, radio=0.0):
        # TODO: Testear
        self.girar_grados(degrees, 'izq', radio, palante=False)

    def girar_grados_der_atras(self, degrees, radio=0.0):
        self.girar_grados(degrees, 'der', radio, palante=False)

    def girar_grados(self, degrees, direccion, radio=0.0, palante=True):
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
        
        if palante:
            vel_lin = self.max_linear_vel
        else:
            vel_lin = -self.max_linear_vel
        
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
                self.publish_wheel_velocity(vel_lin, angular_vel*direccion)
                
                time.sleep(0.1)
                radian_recorridos = angular_vel * 0.1
                radian_total -= abs(radian_recorridos)

            self.detener()

