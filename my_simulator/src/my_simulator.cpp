#include "my_simulator/my_simulator.hpp"

MySimulator::MySimulator()
: Node("my_simulator_node"), // Nombre del nodo en ROS
  x_(0.0), y_(0.0), yaw_(0.0),
  vx_(0.0), vy_(0.0), yaw_rate_(0.0)
{
    // Mensaje para saber que ha arrancado
    RCLCPP_INFO(this->get_logger(), "¡Simulador MySimulator iniciado correctamente!");
    
    // Aquí inicializaremos más cosas en el futuro
}