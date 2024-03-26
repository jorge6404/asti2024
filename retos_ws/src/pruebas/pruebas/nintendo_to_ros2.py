from time import sleep
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
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

from geometry_msgs.msg import TransformStamped

class Button_publisher(Node):

    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_camera_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.publisher_camera_
        self.publisher_
        timer_period = 0.000001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.tupla = (0.0, 0.0)
        self.x = 0.0
        self.y = 0.0
        self.camera_tilt_angle = 0.0
        self.max_camera_tilt = 4.0
        self.min_camera_tilt = -4.0
        self.counter = 0
        
        self.side_counter = False
        
        self.port = '8888'
        
        self.count = 0
        
    def timer_callback(self):
        comando0 = ['ros2', 'run', 'pruebas', 'test_vision_gazebo']
        proceso0 = subprocess.Popen(comando0, preexec_fn=os.setsid) 
        comando = ['sudo', 'tcpdump', '-i', 'wlo1', 'port', self.port, '-X']
        proceso = subprocess.Popen(comando, stdout=subprocess.PIPE)
        for linea in proceso.stdout:
            linea_decodificada = linea.decode('utf-8').strip()
            linea = linea_decodificada.split(' ')
            columna = linea[2][:2]
            print(columna)
        self.run(columna)

		
    def run(self, columna):
        # Ejecutar el comando tcpdump en la terminal
        print('\nListening on port', self.port)
        print('Please be patient, this might take a while\n')
        print('╦╦╦╦╦╦▄▀▀▀▀▀▀▄╦╦╦╦╦╦')
        print('▒▓▒▓▒█╗░░▐░░░╔█▒▓▒▓▒')
        print('▒▓▒▓▒█║░░▐▄▄░║█▒▓▒▓▒')
        print('▒▓▒▓▒█╝░░░░░░╚█▒▓▒▓▒')
        print('╩╩╩╩╩╩▀▄▄▄▄▄▄▀╩╩╩╩╩╩\n')
            
        try:
            print(columna)
            
        
            
                
        except KeyboardInterrupt:
            proceso.terminate()
        self.timer_callback()

def main(args=None):
    rclpy.init(args=args)
    button_pub = Button_publisher()
    rclpy.spin(button_pub)
    button_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

