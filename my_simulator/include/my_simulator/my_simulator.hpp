#ifndef MY_SIMULATOR_HPP
#define MY_SIMULATOR_HPP

// Mensaje personalizado y librerías de posición
#include "rclcpp/rclcpp.hpp"
#include "my_simulator/msg/control.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Clase que hereda de rclcpp::Node
class MySimulator : public rclcpp::Node
{
public:
    // Constructor
    MySimulator();

private:
    // Variables de estado del coche
    double x_;
    double y_;
    double yaw_;    // Orientación
    double v_;  // (m/s )
    double L_;  // Distancia entre los ejes

    // Input del usuario
    double acc_;    // Aceleracion
    double delta_;  // Ángulo volante

    // Para saber cuando dejo de recibir inputs y desacelerar
    rclcpp::Time last_msg_time_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Suscriptor
    rclcpp::Subscription<my_simulator::msg::Control>::SharedPtr control_sub_;

    // Mensaje a RViz
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void update_physics();
    void control_callback(const my_simulator::msg::Control::SharedPtr msg);
};

#endif