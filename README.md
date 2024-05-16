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
  - params
  - rviz
  - scripts
  - src
  - urdf
  - worlds
