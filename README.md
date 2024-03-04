# UJI ROBOTICS - ASTI2024

## Normas de código

- Interesante comentar bien el funcionamiento, el motivo por el que se usa algo...

## Tareas

- Implementar giro con teleop (usando las teclas a y d)
- Mejorar sistema de colcon build para que no dé errores
- Sistema de odometría, [PDF explicativo](test_ws/src/icckinematics.pdf)

## Movimiento de motores   (fase beta_v2)

- Situarse en carpeta test_ws
- Colcon builds (importante hacer orden correcto... esperando script automatizado)
   - colcon build --symlink-install --packages-select bringup custom_interfaces dynamixel_sdk movement
- Source install/setup.bash en cada terminal
- Terminal 1:
  - ros2 run bringup motor_controller
- Terminal 2:
  - ros2 run movement keyboard_teleop
    - Opción para controlar con el teclado: wasd)
  - ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 500}"
    - Donde id es 1 o 2, y velocity puede ser + o -

## Compilación

- cd asti2024/test_ws

(Si están las carpetas build, install y log hay que borrarlas)
- rm -r build
- rm -r install
- rm -r log

- colcon build --packages-select custom_interfaces
- colcon build --packages-select dynamixel_sdk
- source install/setup.bash
- colcon build --symlink-install --packages-select bringup movement
- source install/setup.bash


- cd ../retos_ws

(Si están las carpetas build, install y log hay que borrarlas)
- rm -r build
- rm -r install
- rm -r log

- colcon build --packages-select custom_interfaces
- source install/setup.bash
- colcon build --symlink-install --packages-select semifinal
- source install/setup.bash

Y ya está todo listo mi rey, ya puedes ejecutar los programas como "ros2 run semifinal siguelineas"


