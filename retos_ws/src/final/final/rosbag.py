import rclpy
import time
from final.Movements import Movements

# DISTANCIAS INTRODUCIDAS: EN CM
# GRADOS: EN GRADOS

SLEEP = 0.05       # Tiempo de espera entre movimientos

class MovementSaver():
    def __init__(self):
        self.movements = []     # (movement (w,a,s,d,q,e)
        self.mover = Movements()
        self.fileName = "movements.txt"
        
    def save_inscript(self):
        print("Guardando movimientos...")
        
        with open(self.fileName, "w") as fichero:
            for datos in self.movements:
                cadena = f'{datos[0]},{datos[1]},{datos[2]}\n'
                fichero.write(cadena)
                
        print("Movimientos guardados.")
                
    def load_inscript(self):
        self.movements = []
        with open(self.fileName, "r") as fichero:
            for linea in fichero:
                lista = linea.split(",")
                self.movements.append((lista[0], float(lista[1]), float(lista[2])))
                
    def execute_movement(self, movement, data1, data2):
        if movement == "w":
            self.mover.avanzar_distancia(data1)
        elif movement == "s":
            self.mover.retroceder_distancia(data1)
        elif movement == "a":
            self.mover.girar_grados_izq(data1)
        elif movement == "d":
            self.mover.girar_grados_der(data1)
        elif movement == "q":
            self.mover.girar_grados_izq(data1, data2)
        elif movement == "e":
            self.mover.girar_grados_der(data1, data2)
        
    def execute_movements(self):
        for movement in self.movements:
            self.execute_movement(movement[0], movement[1], movement[2])
            time.sleep(SLEEP)

def main(args=None):
    rclpy.init(args=args)
    
    saver = MovementSaver()
    
    # Programa principal
    opcion = input("1 - Guardar movimientos\n2 - Ejecutar movimientos\n")
    
    if opcion == "1":       # GUARDAR MOVIMIENTOS
        saver.fileName = input("El nombre del fichero: ")
        movement = ""
        while movement != "x":
            # GET MOVEMENT AND DATA
            movement = input("Introduce movimiento (w,a,s,d: Básicos) y (q,e): Giro con radio: ")
            if movement == "w" or movement == "s":
                try:
                    data1 = abs(float(input("Distancia (cm): ")) / 100)
                    data2 = 0
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "a" or movement == "d":
                try:
                    data1 = abs(float(input("Giro (grados): ")))
                    data2 = 0
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "q" or movement == "e":
                try:
                    data1 = abs(float(input("Giro (grados): ")))
                    data2 = abs(float(input("Radio (cm): "))) / 100
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "x":
                saver.save_inscript()
                break
            else:
                print("ERROR MOVIMIENTO")
                continue
            
            # MOVE THE ROBOT WITH ITS DATA
            saver.execute_movement(movement, data1, data2)

            # SAVE THE MOVEMENT
            saver.movements.append((movement, data1, data2))
            
        

    elif opcion == "2":     # EJECUTAR MOVIMIENTOS
        saver.fileName = input("El nombre del fichero: ")
        saver.load_inscript()
        saver.execute_movements()
            
    else:
        print("ERROR OPCION")
        return
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
