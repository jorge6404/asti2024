import rclpy
from rclpy.node import Node
from final.Movements import Movements

# TODO: Configuración de velocidades, poder cambiar entre rondas para así ajustarlo a la mejor puntuación posible.

distancia_inicial = 0.40
grados_izquierda = 90
radio_izquierda = 0.2
distancia_final = 0.25

def posicionarse_patras(mov):
  # TODO: Testear
  # Este es el que rentaría, si la rampa está delante del robot, la cosa sería empezar con el robot al revés.
  print("Posicionando el robot para lanzar los bolos patras")
  mov.retroceder_distancia(distancia_inicial, aceleracion=False)
  mov.girar_grados_der_atras(grados_izquierda, radio=radio_izquierda)        # Derecha realmente es izquierda si fuera hacia adelante
  mov.retroceder_distancia(distancia_final, aceleracion=False)

def posicionarse_palante(mov):
  print("Posicionando el robot para lanzar los bolos palante")
  mov.avanzar_distancia(distancia_inicial)   # Avanzar un poquito para no tocar la pared en el siguiente paso
  mov.girar_grados_izq(grados_izquierda, radio=radio_izquierda)    # Girar y ya avanzar para ponerse en la posición correcta.
  mov.avanzar_distancia(distancia_final)     # Ponerse un poquito más cerca de la linea, asegurarse de no tocarla.
  
def posicionarse_palante_patras(mov):
  # (Modo aparcamiento)
  print("Posicionando el robot para lanzar los bolos palante y patras")
  mov.avanzar_distancia(distancia_inicial*3)  # Hay que pasarse para luego recular
  mov.girar_grados__atras(grados_izquierda, radio=radio_izquierda)
  mov.retroceder_distancia(distancia_final)
  

def lanzar(mov):
  print("Lanzando los bolos")
  mov.bolos_soltar()
  
def lanzar_mientras_retrocede(mov):
  print("Lanzando los bolos mientras retrocede")
  mov.bolos_soltar()
  mov.retroceder_distancia(distancia_final, aceleracion=False)    # Chetar la velocidad aquí para que dé impulso


def main(args=None):
  global distancia_inicial, grados_izquierda, radio_izquierda, distancia_final
  
  rclpy.init(args=args)
  node = rclpy.create_node('bolos')
  mov = Movements()
  
  while True:
    opcion_menu = input("Enter para iniciar el lanzamiento de bolos (q para salir): ")
    if opcion_menu == 'q':
      break
    
    posicionarse_patras(mov)
    lanzar_mientras_retrocede(mov)
    
    distancia_inicial_calib = input(f"\nDistancia inicial actual: {distancia_inicial} m.\nNueva distancia inicial (Enter para omitir): ")
    if distancia_inicial_calib:
      distancia_inicial = float(distancia_inicial_calib)
      
    grados_izquierda_calib = input(f"\nGrados izquierda actual: {grados_izquierda} grados.\nNuevos grados izquierda (Enter para omitir): ")
    if grados_izquierda_calib:
      grados_izquierda = float(grados_izquierda_calib)
    
    radio_izquierda_calib = input(f"\nRadio izquierda actual: {radio_izquierda} m.\nNuevo radio izquierda (Enter para omitir): ")
    if radio_izquierda_calib:
      radio_izquierda = float(radio_izquierda_calib)
    
    distancia_final_calib = input(f"\nDistancia final actual: {distancia_final} m.\nNueva distancia final (Enter para omitir): ")
    if distancia_final_calib:
      distancia_final = float(distancia_final_calib)
    
  rclpy.shutdown()  
  
    
