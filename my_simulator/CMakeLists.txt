cmake_minimum_required(VERSION 3.8)
project(my_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
)

# Incluir directorios (donde está el .hpp)
include_directories(include)

# Crear el ejecutable (uniendo main.cpp y my_simulator.cpp)
add_executable(my_simulator_node 
  src/main.cpp
  src/my_simulator.cpp
)

# Vincular dependencias al ejecutable
ament_target_dependencies(my_simulator_node
  rclcpp
  geometry_msgs
  rosidl_default_runtime
)

# Instalar el ejecutable y carpetas necesarias
install(TARGETS
  my_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
