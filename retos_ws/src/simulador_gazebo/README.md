# Trabajar co el simulador_gazebo

1. Para abrir el simulador de gazebo:
   - `ros2 launch simulador_gazebo launch_sim.launch.py`
2. El simulador te pedira que des el modelo del robot
   tienes dos opciones, (temporalmente hasta que se actualice el simulador):

   - `1) rsp.launch.py (para el robot con la camara hacia abajo)`
   - `2) rsp_front.launch.py (para el robot con la camara hacia adelante)`

3. Para usar un world especifico:
    - `ros2 launch simulador_gazebo launch_sim.launch.py world:=dirección_del_mundo`
    - Ejemplo: `ros2 launch simulador_gazebo launch_sim.launch.py world:=./src/simulador_gazebo/worlds/cuadricula.world`
   ### Nota:
    - Si se quiere usar un mundo que no esté en la carpeta `simulador_gazebo/worlds`, habrá que poner la dirección completa del mundo.
    - Se debe tener en cuenta que el mundo debe ser un archivo `.world` y que debe estar en la carpeta `simulador_gazebo/worlds`.
    - Tambien se debe de poner el ./src/ para que el simulador lo encuentre, (si estas ejecutando el codigo desde el ws).

4. Para mover el robot:
    - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

5. Modificaciones en el mundo:
    - Para modificar el mundo, se debe de modificar el archivo `.world` que se encuentra en la carpeta `simulador_gazebo/worlds`.
    - Para modificar el robot, se debe de modificar el archivo `.xacro` que se encuentra en la carpeta 
    - `simulador_gazebo/models/modelo a elegir/robot_core.xacro`.
    - Para modificar la camara, se debe de modificar el archivo `.xacro` que se encuentra en la carpeta
    - `simulador_gazebo/models/modelo a elegir/camera.xacro`.

