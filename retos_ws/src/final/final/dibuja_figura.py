#!/usr/bin/env python3

MODO_TESTEO=True

# Import the necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import GPIO for Raspberry Pi
if not MODO_TESTEO:
  import RPi.GPIO as GPIO
import time
from math import pi

# Constantes
SERVO_PIN = 12
FRECUENCIA = 50
SLEEP_TIME_BOLI = 0.5
SLEEP_TIME_MOVIMIENTO = 0.1


# Servo y GPIO
def init_servo():
  # GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(12, GPIO.OUT)
  servo = GPIO.PWM(12, 50)
  servo.start(0)
  return servo

# Nodo, publicador y mensaje
def init_ros():
  rclpy.init(args=None)
  node = rclpy.create_node('figuras')
  cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
  cmd_vel_msg = Twist()
  return node, cmd_vel_pub, cmd_vel_msg

def get_figure_params():
  figura = int(input("Figura (1: Triangulo, 2: Cuadrado, 3: Rectangulo): "))
  if figura == 1: # Triangulo
    angulo = 2.09439510 # 2pi/3
    largo = 0.25
    ancho = largo
    lados = 3
  elif figura == 2: # Cuadrado
    angulo = 1.57079632 # 2pi/4
    largo = 0.2
    ancho = largo
    lados = 4
  elif figura == 3: # Rectangulo
    angulo = 1.57079632 # 2pi/4
    largo = 0.3
    ancho = 0.1
    lados = 4
  else:
    raise ValueError("Invalid figure")
  return angulo, largo, ancho, lados


def main():
  if not MODO_TESTEO:
    servo = init_servo()
  node, cmd_vel_pub, cmd_vel_msg = init_ros()
  angulo, largo, ancho, lados = get_figure_params()

  if not MODO_TESTEO:
    servo.ChangeDutyCycle(2+(130/18))
  else:
    print("Bajando boli (??)")
    
  velocidad_lineal = -0.1
  velocidad_angular = 0.2

  for i in range(lados):
      if i % 2:
          distancia = largo
      else:
          distancia = ancho
      
      # Move forward
      print("Moviendo hacia adelante")
      for linear_iterations in range(int(distancia / (abs(velocidad_lineal) * 0.1))):
          cmd_vel_msg.linear.x = velocidad_lineal
          cmd_vel_msg.angular.z = 0.0
          cmd_vel_pub.publish(cmd_vel_msg)
          time.sleep(SLEEP_TIME_MOVIMIENTO)

      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      cmd_vel_pub.publish(cmd_vel_msg)

      # Pen up
      if not MODO_TESTEO:
        servo.ChangeDutyCycle(2+(110/18))
      else:
        print("Subiendo boli")
      time.sleep(SLEEP_TIME_BOLI)

      # Move forward 16cm
      print("Moviendo hacia adelante")
      for linear_iterations in range(int(0.2 / (abs(velocidad_lineal) * 0.1))):
          cmd_vel_msg.linear.x = velocidad_lineal
          cmd_vel_msg.angular.z = 0.0
          cmd_vel_pub.publish(cmd_vel_msg)
          time.sleep(SLEEP_TIME_MOVIMIENTO)

      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      cmd_vel_pub.publish(cmd_vel_msg)

      # Turn
      print("Girando")
      for angular_iterations in range(int(angulo / (abs(velocidad_angular) * 0.1))):
          cmd_vel_msg.linear.x = 0.0
          cmd_vel_msg.angular.z = velocidad_angular
          cmd_vel_pub.publish(cmd_vel_msg)
          time.sleep(SLEEP_TIME_MOVIMIENTO)

      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      cmd_vel_pub.publish(cmd_vel_msg)

      # Move backward 16cm
      print("Moviendo hacia atras")
      for linear_iterations in range(int(0.15 / (abs(velocidad_lineal) * 0.1))):
          cmd_vel_msg.linear.x = -velocidad_lineal
          cmd_vel_msg.angular.z = 0.0
          cmd_vel_pub.publish(cmd_vel_msg)
          time.sleep(SLEEP_TIME_MOVIMIENTO)

      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      cmd_vel_pub.publish(cmd_vel_msg)

      # Bajar boli
      if not MODO_TESTEO:
        servo.ChangeDutyCycle(2+(130/18))
      else:
        print("Bajando boli")
      time.sleep(SLEEP_TIME_BOLI)
      
  if not MODO_TESTEO:
    servo.ChangeDutyCycle(2+(120/18))
    servo.stop()
    GPIO.cleanup()

  rclpy.shutdown()
