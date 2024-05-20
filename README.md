# Quetzalcoders Final Project Simulation

## Contributors:
- **Angel Antonio Sánchez Medina** (A01412516)
- **Emiliano Mendoza Nieto** (A01706083)
- **Diego Alfonso Ramirez Montes** (A01707596)
- **Javier Suárez Durán** (A01707380)
- **Jose Angel García Lopez** (A01275108)

============================================================================================================

Este repositorio se encuentra construido de la siguiente forma: 

  - gazebo
  - include
  - launch
    - include
      - robot
        - control.launch: Contiene los argumentos de odometria y descripcion del robot para sus movimientos.
        - description.launch: Llama a la carpeta urdf.
        - spawner.launch: Spawnea el Puzzlebot en el Gazebo.
      - gazebo.launch: Inicia el mapa en gazebo.
    - simulation.launch: Es el archivo launch que nos ayuda a inicializar todo el proyecto, como ejecutable.                             Los nodos, topicos, y llamar a los demas elementos como rviz, gazebo, el robot, el                              mapa, etc.
  - models:
    - camera.stl
    - MCR2_1000_1_1_Wheel_Coupler_2.stl
    - MCR2_1000_13_Chassis.stl
    - Puzzlebot.stl
  - params
  - rviz:
    - maping_conf_rviz.rviz
    - navigation_conf_rviz.rviz
  - scripts: Aqui es donde se encontraran nuestros codigos desarrollados para cumplir con los                               tasks/challenges solicitados.
    - chassisMovement.py
    - puzzlebot_odometry.py
    - wheelsMovement.py
  - src
  - urdf:
    - puzzlebot.urdf
  - worlds:
    - ADJIC_map.world: Es el mapa

============================================================================================================

INSTRUCCIONES PARA CORRER EL PROYECTO: 

1.- git clone (del repositorio en la carpeta de trabajo)


2.- cd catkin_ws (Carpeta de trabajo)


3.- catkin_make


4.- cd src


5.- cd quetzalcoders_autonomousnavigation_puzzlebot


6.- roslaunch quetzalcoders_autonomousnavigation_puzzlebot simulation.launch
  







