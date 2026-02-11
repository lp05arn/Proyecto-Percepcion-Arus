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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  DEPENDENCIES std_msgs
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
  std_msgs
  visualization_msgs
  rosidl_default_runtime
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_target_interfaces(my_simulator_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Instalar el ejecutable y carpetas necesarias
install(TARGETS
  my_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tracks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
