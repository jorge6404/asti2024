# Source original: https://gist.github.com/flyboy74/21e38dec4bbb77eaf8d729e6deba8d14
# Videos del pibe: 
## https://www.youtube.com/watch?v=AO7NFmX6klQ
## (lo raro es que no es indio, pinta que es un inglés puro del centro de gran bretaña)

# TODO: Intersecciones da mal
# todo: Probar en fisico

import time
import cv2
import numpy as np

# Constants
ROI_START_Y = 200
ROI_END_Y = 250
ROI_START_X = 0
ROI_END_X = 639
BLACKLINE_COLOR_RANGE_LOW = (0, 0, 0)
BLACKLINE_COLOR_RANGE_HIGH = (50, 50, 50)
KERNEL_SIZE = (3, 3)
EROSION_ITERATIONS = 5
DILATION_ITERATIONS = 9
LINE_COLOR = (255, 0, 0)
LINE_THICKNESS = 3
SETPOINT = 320

# Position memory for contour selection
x_last = 320
y_last = 180

# Open the camera
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Could not open camera")
    exit(1)

# Allow the camera to warm up
time.sleep(0.1)

try:
    while True:
        # Capture frame-by-frame
        ret, image = camera.read()
        if not ret:
            break

        # Define region of interest
        # roi = image[ROI_START_Y:ROI_END_Y, ROI_START_X:ROI_END_X]

        # Detect black line
        blackline_mask = cv2.inRange(image, BLACKLINE_COLOR_RANGE_LOW, BLACKLINE_COLOR_RANGE_HIGH)    # Use roi to improve performance
        kernel = np.ones(KERNEL_SIZE, np.uint8)
        blackline_mask = cv2.erode(blackline_mask, kernel, iterations=EROSION_ITERATIONS)
        blackline_mask = cv2.dilate(blackline_mask, kernel, iterations=DILATION_ITERATIONS)

        # Find contours
        contours_blk, _ = cv2.findContours(blackline_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk_len = len(contours_blk)


        if contours_blk_len > 0:
            print("\n\n")
            print(f"Numero de contornos: {contours_blk_len}")
            
            if contours_blk_len == 1:
                blackbox = cv2.minAreaRect(contours_blk[0])
            else:
                candidates = []
                off_bottom = 0
                
                # Comprobacion de todas las lineas
                for con_num in range(contours_blk_len):
                    blackbox = cv2.minAreaRect(contours_blk[con_num])
                    (x_min, y_min), (w_min, h_min), ang = blackbox
                    box = cv2.boxPoints(blackbox)
                    (x_box, y_box) = box[0]
                    if y_box > 358:
                        off_bottom += 1
                    candidates.append((y_box, con_num, x_min, y_min))
                candidates = sorted(candidates)     # Ordenar por posicion en y
                
                if off_bottom > 1:
                    candidates_off_bottom = []
                    for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                        (y_highest, con_highest, x_min, y_min) = candidates[con_num]
                        total_distance = (abs(x_min - x_last) ** 2 + abs(y_min - y_last) ** 2) ** 0.5
                        candidates_off_bottom.append((total_distance, con_highest))
                    candidates_off_bottom = sorted(candidates_off_bottom)
                    (total_distance, con_highest) = candidates_off_bottom[0]
                    blackbox = cv2.minAreaRect(contours_blk[con_highest])
                else:
                    (y_highest, con_highest, x_min, y_min) = candidates[contours_blk_len - 1]
                    blackbox = cv2.minAreaRect(contours_blk[con_highest])
            
                print(f"Contorno seleccionado: {con_highest}\n-y_highest: {y_highest}\n-x_min: {x_min}\n-y_min: {y_min}")
                
            (x_min, y_min), (w_min, h_min), ang = blackbox
            x_last = x_min
            y_last = y_min
            # cv2.drawContours(image, contours_blk, -1, (0, 255, 0), 3)
            
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            error = int(x_min - SETPOINT)
            ang = int(ang)
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            
            cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            
            cv2.line(image, (int(x_min), ROI_START_Y), (int(x_min), ROI_END_Y), LINE_COLOR, LINE_THICKNESS)
            

        # Display the resulting frame
        cv2.imshow("Original with line", image)


        # ------------ CÁLCULO DE LA VELOCIDAD DE LOS MOTORES (en base al angulo)
        # Informacion
        # - Angulo proximo a 90 grados: robot recto
        # - Angulo positivo: giro a la izquierda, a mayor diferencia entre angulo y 90, mayor giro
        # - Angulo negativo: giro a la derecha, a mayor diferencia entre abs(angulo y 90), mayor giro
        
        # KP_ANGULO = 1.0  # Ganancia proporcional para el error de ángulo
        VELOCIDAD_BASE = 50
        
        # Calculo de la velocidad de los motores
        dif_ang = abs(ang) - 90
        print(f"\nAngulo: {ang}")
        print(f"Diferencia de angulo: {dif_ang}")
        
        if ang < 0:
            print("Giro a la derecha")
            velocidad_motor_izquierdo = VELOCIDAD_BASE + abs(dif_ang)
            velocidad_motor_derecho = VELOCIDAD_BASE
        else:
            print("Giro a la izquierda")
            velocidad_motor_izquierdo = VELOCIDAD_BASE
            velocidad_motor_derecho = VELOCIDAD_BASE + abs(dif_ang)
            
        print('---')
        print(f"Velocidad motor izquierdo: {velocidad_motor_izquierdo}")
        print(f"Velocidad motor derecho: {velocidad_motor_derecho}")
        print('---')
        
        # ----------- V2: CÁLCULO DE LA VELOCIDAD DE LOS MOTORES (en base al angulo y al error)
        
        # Informacion
        # - Error proximo a 0: robot recto
        # - Error negativo: linea a la izquierda, girar izquierda
        # - Error positivo: linea a la derecha, girar derecha
        
        # TODO: Implementarlo esto
        
        time.sleep(0.1)
        


        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # When everything done, release the capture
    camera.release()
    cv2.destroyAllWindows()
