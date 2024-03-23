#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi
from .Movements import Movements

SLEEP_TIME_BOLI = 0.5
SLEEP_TIME_MOVIMIENTO = 0.1

VELOCIDAD_LINEAL = 0.1
VELOCIDAD_ANGULAR = 0.5

def subir_boli():
  print("Subiendo boli")

def bajar_boli():
  print("Bajando boli")


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
  print(" q. Salir")
  
  return input("Seleccione una opcion: ")

def dibujar_figura(mov: Movements, opcion_menu):
  angulo, largo, ancho, lados = get_figure_params(opcion_menu)
  
  bajar_boli()
  
  for i in range(lados):
    if i % 2:
        distancia = largo
    else:
        distancia = ancho
    
    # Move forward
    print("Moviendo hacia adelante")
    for linear_iterations in range(int(distancia / (abs(VELOCIDAD_LINEAL) * SLEEP_TIME_MOVIMIENTO))):
        mov.avanzar()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()
    subir_boli()

    # Move forward 16cm
    print("Moviendo hacia adelante")
    for linear_iterations in range(int(0.2 / (abs(VELOCIDAD_LINEAL) * SLEEP_TIME_MOVIMIENTO))):
        mov.avanzar()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()

    # Turn
    print("Girando")
    for angular_iterations in range(int(angulo / (abs(VELOCIDAD_ANGULAR) * SLEEP_TIME_MOVIMIENTO))):
        mov.girar_izquierda()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()

    # Move backward 16cm
    print("Moviendo hacia atras")
    for linear_iterations in range(int(0.15 / (abs(VELOCIDAD_LINEAL) * SLEEP_TIME_MOVIMIENTO))):
        mov.retroceder()
        time.sleep(SLEEP_TIME_MOVIMIENTO)

    mov.detener()
    bajar_boli()



def main():
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
      subir_boli()
    elif opcion_menu == 'd':
      bajar_boli()
    elif opcion_menu == 'q':
      break
    # Dibujar figura
    elif opcion_menu in ['1', '2', '3']:  # Triangulo, cuadrado, rectangulo
      dibujar_figura(mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

