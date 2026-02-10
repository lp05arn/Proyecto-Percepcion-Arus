cmake_minimum_required(VERSION 3.8)
project(my_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 2. Incluir directorios (donde está el .hpp)
include_directories(include)

# 3. Crear el ejecutable (uniendo main.cpp y my_simulator.cpp)
add_executable(my_simulator_node 
  src/main.cpp
  src/my_simulator.cpp
)

# 4. Vincular dependencias al ejecutable
ament_target_dependencies(my_simulator_node rclcpp)

# 5. Instalar el ejecutable y carpetas necesarias
install(TARGETS
  my_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
