# UJI ROBOTICS - ASTI2024


## ¿Cómo ejecutar un programa en el CyberCrex?

### Opción 100% Raspberry Pi

1. Conectar raspberry a la corriente / batería.
2. Entrar en la raspberry con ssh desde otro portátil conectado a la misma red wifi.
  - `ssh -X pi@192.168.0.114` (Contraseña: qwerty)
  - La red wifi puede ser el router PIROBOTNET o los datos de cualquier móvil ya configurado.
  - Para conectarlo a los datos de un móvil o red no configurada, habría que hacerlo desde la interfaz gráfica de la raspberry, conectando un teclado, ratón y monitor... O conectando cable ethernet.

3. Sources.
  - `cd asti2024/retos_ws`
  - `source install/setup.bash`  (O usar el alias `src` para hacerlo más rápido)

4. Ejecutar el programa deseado.
  - `ros2 run bringup motor_vel_controller`     (Para activar los motores, no se debe cerrar esta terminal. Asegurarse de que está los motores están bloqueados girando las ruedas con la mano)
  - `ros2 run final dibuja_figura`  (En otra terminal, aquí sería donde se ejecutaría el programa deseado)

5. En caso de error:
  - Hacer sources:
    - `source /opt/ros/foxy/setup.bash`
    - `source install/setup.bash`
  - Compilar el código en la raspberry, en principio ya estará siempre hecho.
    - `colcon build --packages-select custom_interfaces`
    - `colcon build --packages-select dynamixel_sdk`
    - `source install/setup.bash`
    - `colcon build --symlink-install --packages-select bringup`
    - `source install/setup.bash`
  - Si sigue sin funcionar
    - `sudo usermod -aG dialout pi`
    - `sudo chmod 777 /dev/ttyUSB0`  (o el puerto que sea, podría ser ttyUSB1, ttyUSB2, etc. Usar `ls /dev/ttyUSB*` para verlo)
    - Reiniciar la controladora de motores.
    - Cambiar el usb de sitio.

6. Para cerrar la raspberry:
  - `sudo shutdown now`  (Esperar a que se apague la luz roja del led de la raspberry, no desconectar la corriente antes de que se apague)

7. Para cerrar la conexión ssh:
  - `exit`


### Testeos en el portátil

1. Descargar repositorio.
2. Compilar el código
  - `colcon build --packages-select custom_interfaces`
  - `colcon build --packages-select dynamixel_sdk`
  - `source install/setup.bash`
  - `colcon build --symlink-install --packages-select bringup`
  - `source install/setup.bash`


## Ejecución

MOTORES

Terminal 1 (Donde se ha compilado todo):  
`ros2 run bringup motor_vel_controller`

En caso de fallo:  
`sudo usermod -aG dialout <linux_account>`  
`sudo chmod 777 /dev/ttyUSB0`  
`ros2 run bringup motor_vel_controller /dev/ttyUSB0`   (usar `ls /dev/ttyUSB*` para verlo)  
Reiniciar la controladora de motores.  
Cambiar el usb de sitio.

Terminal 2:   
`source /opt/ros/foxy/setup.bash`  
`source install/setup.bash`  
`ros2 run semifinal siguelineas`  (o laberinto)




PRUEBAS

`cd ../retos_ws`

(Si están las carpetas build, install y log hay que borrarlas)  
`rm -r build`  
`rm -r install`  
`rm -r log`  

`colcon build --packages-select custom_interfaces`  
`source install/setup.bash`  
`colcon build --symlink-install --packages-select semifinal`  
`source install/setup.bash`





### Otros

`ros2 topic pub cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

`ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 50}"`  
(Id puede ser 1 o 2, y velocity puede ser + o -)

`ros2 run movement keyboard_teleop`

