--Simulador de coche desarrollado en C++ y ROS 2 (también RViz) para pruebas de algoritmos de conducción autónoma en Formula Student--

-Simula la dinámica de un coche, junto con la carga de conos (circuito) y genera una capa de percepción simulada (LiDAR) con ruido gaussiano y campo de visión limitado-

# Funciones Principales

- Dinámica del vehículo (modelo bicicleta)
- Carga de circuito: Lectura de posiciones de conos
- Simulación de percepción LiDAR: Campo de visión 180º, rango máximo 20m, ruido gaussiano
- Visualización en RViz

# Tecnologías usadas

- C++
- ROS2
- RViz2
- CMake
- Git

## INSTALACIÓN ##
1. Clonar repositorio en un workspace de ROS2
2. Instalar dependencias (rosdep install --from-paths src --ignore-src -r -y)
3. Compilar el paquete
   3.1 colcon build --packages-select my_simulator
   3.2 source install/setup.bash

## USO ##
1. Lanzar el nodo principal desde terminal: ros2 run my_simulator my_simulator_node
2. Visualizar en RViz
   - Fixed Frame: map
   - MarkerArray (Realidad): Topic /track_markers (Añadiendolo si no aparece)
   - MarkerArray (Percepción): Topic /perception_markers (Añadiendolo si no aparece)
3. Ya podemos mover el coche con comandos directos o usando el teclado con su debida instalación. (ej: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.5}})

## ESTRUCTURA ##
my_simulator/
├── config/             # Archivos de configuración (circuitos)
├── launch/             # Launchfiles (opcional)
├── src/
│   └── my_simulator.cpp # Código fuente principal (Lógica del nodo)
├── CMakeLists.txt      # Configuración de compilación
├── package.xml         # Dependencias del paquete
└── README.md           # Documentación


Autor: lp05arn | Alejandro Ruiz Navarrete
Licencia: MIT
