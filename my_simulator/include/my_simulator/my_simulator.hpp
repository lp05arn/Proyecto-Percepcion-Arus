#ifndef MY_SIMULATOR_HPP
#define MY_SIMULATOR_HPP

#include "rclcpp/rclcpp.hpp"

// Clase que hereda de rclcpp::Node
class MySimulator : public rclcpp::Node
{
public:
    // Constructor
    MySimulator();

private:
    // Variables de estado del coche (según el ejercicio)
    double x_;
    double y_;
    double yaw_;      // Orientación
    double vx_;       // Velocidad en X
    double vy_;       // Velocidad en Y
    double yaw_rate_; // Velocidad de giro
};

#endif // MY_SIMULATOR_HPP