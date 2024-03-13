#!/usr/bin/env python3
# TODO: Implementar giros con "Movements.py" en lugar de hacerlo manualmente?

CON_GPIO=False   # Cambiar a True si se está ejecutando en la Raspberry Pi con el motor de la herramienta

# Import the necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
if CON_GPIO:
  import RPi.GPIO as GPIO
import time
from math import pi
from .Movements import Movements

SERVO_PIN = 12 
FRECUENCIA = 50
SLEEP_TIME_BOLI = 0.5
SLEEP_TIME_MOVIMIENTO = 0.1

VELOCIDAD_LINEAL = 0.1
VELOCIDAD_ANGULAR = 0.5



# Servo y GPIO
def init_servo():
  if CON_GPIO:
    print("Inicializando servo")
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo = GPIO.PWM(SERVO_PIN, FRECUENCIA)
    servo.start(0)
    time.sleep(1)   # Esperar a que el servo se inicialice
    print("Servo inicializado correctamente")
    return servo
  else:
    print("Servo no detectado para inicializarlo.")
    return None

def parar_servo(servo):
  if CON_GPIO:
    print("Parando servo")
    servo.ChangeDutyCycle(2+(120/18))
    servo.stop()
    GPIO.cleanup()
  else:
    print("Servo no detectado para pararlo.")
  
def subir_boli(servo):
  if CON_GPIO:
    print("Subiendo boli")
    servo.ChangeDutyCycle(2+(110/18))       # TODO: Revisar valores y cambiar posiciones servos
  else:
    print("Boli no detectado para subrirlo.")
  time.sleep(SLEEP_TIME_BOLI)

def bajar_boli(servo):
  if CON_GPIO:
    print("Bajando boli")
    servo.ChangeDutyCycle(2+(130/18))
  else:
    print("Boli no detectado para bajarlo.")
  time.sleep(SLEEP_TIME_BOLI)


def get_figure_params(opcion_menu):
  figura = int(opcion_menu)
  
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

def pedir_opcion_menu():
  print("\nFiguras principales")
  print(" 1. Triangulo")
  print(" 2. Cuadrado")
  print(" 3. Rectangulo")
  print("\nPruebas:")
  print(" a. Prueba de movimiento")
  print(" s. Subir boli")
  print(" d. Bajar boli")
  print(" f. Reiniciar servo")
  print(" q. Salir")
  
  return input("Seleccione una opcion: ")

def dibujar_figura(servo, mov: Movements, opcion_menu):
  angulo, largo, ancho, lados = get_figure_params(opcion_menu)
  
  bajar_boli(servo)
  
  for i in range(lados):
    if i % 2:
        distancia = largo
    else:
        distancia = ancho
    
    # Move forward
    print("Moviendo hacia adelante")
    for linear_iterations in range(int(distancia / (abs(VELOCIDAD_LINEAL) * 0.1))):
        mov.avanzar()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()
    subir_boli(servo)

    # Move forward 16cm
    print("Moviendo hacia adelante")
    for linear_iterations in range(int(0.2 / (abs(VELOCIDAD_LINEAL) * 0.1))):
        mov.avanzar()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()

    # Turn
    print("Girando")
    for angular_iterations in range(int(angulo / (abs(VELOCIDAD_ANGULAR) * 0.1))):
        mov.girar_izquierda()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()

    # Move backward 16cm
    print("Moviendo hacia atras")
    for linear_iterations in range(int(0.15 / (abs(VELOCIDAD_LINEAL) * 0.1))):
        mov.retroceder()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()
    bajar_boli(servo)



def main():
  servo = init_servo()
  
  rclpy.init(args=None)
  node = rclpy.create_node('figuras')
  mov = Movements()
  mov.actualizar_velocidades(VELOCIDAD_LINEAL, VELOCIDAD_ANGULAR, 0.01, 0.1)    # TODO: Aceleraciones mejorar
  
  while True:
    opcion_menu = pedir_opcion_menu()
    
    # Testeos
    if opcion_menu == 'a':
      mov.prueba_movimientos()
    elif opcion_menu == 's':
      subir_boli(servo)
    elif opcion_menu == 'd':
      bajar_boli(servo)
    elif opcion_menu == 'f':
      parar_servo(servo)
      servo = init_servo()
    elif opcion_menu == 'q':
      break
    # Dibujar figura
    elif opcion_menu in ['1', '2', '3']:  # Triangulo, cuadrado, rectangulo
      dibujar_figura(servo, mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  parar_servo(servo)
  rclpy.shutdown()
