import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Button_publisher(Node):

    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.tupla = (0.0, 0.0)
        self.x = 0.0
        self.y = 0.0
        self.counter = 0
        
        self.side_counter = False
        
    def timer_callback(self):
        self.run()
		
    def run(self):
    # Ejecutar el comando tcpdump en la terminal
        comando = ['sudo', 'tcpdump', '-i', 'wlo1', 'port', '8888', '-X']
        proceso = subprocess.Popen(comando, stdout=subprocess.PIPE)

        try:
        # Leer la salida línea por línea
            
            for linea in proceso.stdout:
                # Decodificar la línea a UTF-8
                linea_decodificada = linea.decode('utf-8').strip()
                linea = linea_decodificada.split(' ')
                columna = linea[2]
                if columna == '0000' and self.counter == 0:
                    print('Start!')
                    self.side_counter = False
                    self.counter += 1
                elif columna == '0100': # A
                    self.tupla = (0.2, 0.0)
                    self.x = 0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                elif columna == '0200': # B
                    self.tupla = (-0.2, 0.0)
                    self.x = -0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                elif columna == '0004': # X
                    self.tupla = (0.0, 0.0)
                    self.x = 0.0
                    self.y = 0.0
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                elif columna == '0008': # Y
                    pass
                elif columna == '4000' and self.x <= 1.0 and self.side_counter == False: # UP
                    self.side_counter = True
                    self.x += 0.1
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                    
                elif columna == '8000' and self.x >= -1.0 and self.side_counter == False: # DOWN
                    self.side_counter = True
                    self.x -= 0.1
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                    
                elif columna == '2000' and self.y <= 1.0 and self.side_counter == False: # LEFT
                    self.side_counter = True
                    self.y += 0.1
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                    
                elif columna == '1000' and self.y >= -1.0 and self.side_counter == False: # RIGHT
                    self.side_counter = True
                    self.y -= 0.1
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                    
                elif columna == '0001': # R
                    pass
                elif columna == '0002': # L
                    pass  	
            
                
        except KeyboardInterrupt:
            # Manejar la interrupción de teclado para terminar el proceso
            proceso.terminate()

def main(args=None):
    rclpy.init(args=args)
    button_pub = Button_publisher()
    rclpy.spin(button_pub)
    button_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

