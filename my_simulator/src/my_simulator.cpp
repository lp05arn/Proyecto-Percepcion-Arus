#include "my_simulator/my_simulator.hpp"

using namespace std::chrono_literals;

MySimulator::MySimulator()
: Node("my_simulator_node"), 
  x_(0.0), y_(0.0), yaw_(0.0),
  v_(0.0), acc_(0.0), delta_(0.0)
{
    L_ = 2.5; // Distancia entre los ejes de 2.5 metros.

    // Inicializamos el suscriptor
    control_sub_ = this->create_subscription<my_simulator::msg::Control>(
      "/car_control", 10, std::bind(&MySimulator::control_callback, this, std::placeholders::_1)
    );
    
    // Inicializamos RViz y timer
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MySimulator::update_physics, this)
    );

    // Mensaje de "arranque"
    RCLCPP_INFO(this->get_logger(), "¡Simulador MySimulator iniciado correctamente!");
}

// Guardamos los inputs del ususario
void MySimulator::control_callback(const my_simulator::msg::Control::SharedPtr msg)
{
  acc_ = msg->acc;
  delta_ = msg->delta;
}

// Calcular físicas
void MySimulator::update_physics()
{
  double dt = 0.1;

  v_ += acc_*dt;
  x_ += (v_ * cos(yaw_))*dt;
  y_ += (v_ * sin(yaw_))*dt;
  yaw_ += (v_ / L_) * tan(delta_)*dt;

  // Lo pasamos RViz
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Enviamos el "mensaje"
  tf_broadcaster_->sendTransform(t);
}