#ifndef MY_SIMULATOR_HPP
#define MY_SIMULATOR_HPP

// Mensaje personalizado y librerías de posición
#include "rclcpp/rclcpp.hpp"
#include "my_simulator/msg/control.hpp"
#include "my_simulator/msg/cone.hpp"
#include "my_simulator/msg/cone_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <random>

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

    // Datos del circuito
    std::vector<my_simulator::msg::Cone> track_cones_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Suscriptor
    rclcpp::Subscription<my_simulator::msg::Control>::SharedPtr control_sub_;

    // Mensaje a RViz
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_pub_;

    rclcpp::Publisher<my_simulator::msg::ConeArray>::SharedPtr perception_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perception_viz_pub_;

    // Generador de ruido
    std::default_random_engine noise_gen_;
    std::normal_distribution<double> noise_dist_;

    void update_physics();
    void update_perception();
    void control_callback(const my_simulator::msg::Control::SharedPtr msg);

    // Leer y dibujar el archivo
    void load_circuit(std::string path); // Lee el CSV
    void publish_track();
};

#endif