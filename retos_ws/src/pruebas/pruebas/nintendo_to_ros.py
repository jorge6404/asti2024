import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import sys
import os
import signal

class Button_publisher(Node):

    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera_tilt_publisher = self.create_publisher(Float64, 'camera_tilt_angle', 10)

        
        self.tupla = (0.0, 0.0)
        self.x = 0.0
        self.y = 0.0
        self.camera_tilt_angle = 0.0
        self.max_camera_tilt = 4.0
        self.counter = 0
        
        self.side_counter = False
        
        self.port = '8888'
        
    def timer_callback(self):
        self.run()
		
    def run(self):
        # Ejecutar el comando tcpdump en la terminal
        print('\nListening on port', self.port)
        print('Please be patient, this might take a while\n')
        print('╦╦╦╦╦╦▄▀▀▀▀▀▀▄╦╦╦╦╦╦')
        print('▒▓▒▓▒█╗░░▐░░░╔█▒▓▒▓▒')
        print('▒▓▒▓▒█║░░▐▄▄░║█▒▓▒▓▒')
        print('▒▓▒▓▒█╝░░░░░░╚█▒▓▒▓▒')
        print('╩╩╩╩╩╩▀▄▄▄▄▄▄▀╩╩╩╩╩╩\n')
        comando0 = ['ros2', 'run', 'pruebas', 'test_vision_gazebo']
        proceso0 = subprocess.Popen(comando0, preexec_fn=os.setsid) 
        comando = ['sudo', 'tcpdump', '-i', 'wlo1', 'port', self.port, '-X']
        proceso = subprocess.Popen(comando, stdout=subprocess.PIPE)      
        try:
            # Leer la salida línea por línea
            for linea in proceso.stdout:
                # Decodificar la línea a UTF-8
                linea_decodificada = linea.decode('utf-8').strip()
                linea = linea_decodificada.split(' ')
                columna = linea[2]
                if columna == '0000' and self.counter == 0:
                    print('\n\n░░░░░░░░░░░░░░░░░░░░░░█████████░░░░░░░░░')
                    print('░░███████░░░░░░░░░░███▒▒▒▒▒▒▒▒███░░░░░░░')
                    print('░░█▒▒▒▒▒▒█░░░░░░░███▒▒▒▒▒▒▒▒▒▒▒▒▒███░░░░')
                    print('░░░█▒▒▒▒▒▒█░░░░██▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░░')
                    print('░░░░█▒▒▒▒▒█░░░██▒▒▒▒▒██▒▒▒▒▒▒██▒▒▒▒▒███░')
                    print('░░░░░█▒▒▒█░░░█▒▒▒▒▒▒████▒▒▒▒████▒▒▒▒▒▒██')
                    print('░░░█████████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██')
                    print('░░░█▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒██')
                    print('░██▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒██▒▒▒▒▒▒▒▒▒▒██▒▒▒▒██')
                    print('██▒▒▒███████████▒▒▒▒▒██▒▒▒▒▒▒▒▒██▒▒▒▒▒██')
                    print('█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒████████▒▒▒▒▒▒▒██')
                    print('██▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░')
                    print('░█▒▒▒███████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░░░')
                    print('░██▒▒▒▒▒▒▒▒▒▒████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░░░░░')
                    print('░░████████████░░░█████████████████░░░░░░')
                    print('\n##########')
                    print('# START! #')
                    print('##########\n')
                    self.counter += 1
                elif columna == '0000':
                    self.side_counter = False
                elif columna == '0100': # A
                    self.tupla = (0.2, 0.0)
                    self.x = 0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                elif columna == '0200': # B
                    self.tupla = (-0.2, 0.0)
                    self.x = -0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                elif columna == '0004': # X
                    self.tupla = (0.0, 0.0)
                    self.x = 0.0
                    self.y = 0.0
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                elif columna == '0008': # Y
                    pass
                elif columna == '4000' and self.x <= 2.0 and self.side_counter == False: # UP
                    self.side_counter = True
                    self.y = 0.0
                    self.x += 0.2
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '8000' and self.x >= -2.0 and self.side_counter == False: # DOWN
                    self.side_counter = True
                    self.y = 0.0
                    self.x -= 0.2
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '2000' and self.y <= 2.0 and self.side_counter == False: # LEFT
                    self.side_counter = True
                    self.x = 0.0
                    self.y += 0.3
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '1000' and self.y >= -2.0 and self.side_counter == False: # RIGHT
                    self.side_counter = True
                    self.x = 0.0
                    self.y -= 0.3
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '0001': # R
                    if self.camera_tilt_angle < self.max_camera_tilt:
                        self.camera_tilt_angle += 0.1 
                        camera_msg = Float64()
                        camera_msg.data = self.camera_tilt_angle
                        self.camera_tilt_publisher.publish(camera_msg)
                        self.get_logger().info(f"{camera_msg.data}")
                elif columna == '0002': # L
                    pass  
                elif columna == '0800': # START
                    print('Shutting down...')
                    os.killpg(os.getpgid(proceso0.pid), signal.SIGTERM)
                    
                    exit()
            
                
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

