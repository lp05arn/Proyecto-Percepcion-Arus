#include "my_simulator/my_simulator.hpp"

using namespace std::chrono_literals;

MySimulator::MySimulator()
: Node("my_simulator_node"), 
  x_(0.0), y_(0.0), yaw_(0.0),
  v_(0.0), acc_(0.0), delta_(0.0)
{
    L_ = 2.5; // Distancia entre los ejes de 2.5 metros.

    // Inicializamos reloj y el suscriptor
    last_msg_time_ = this->now();
    control_sub_ = this->create_subscription<my_simulator::msg::Control>(
      "/car_control", 10, std::bind(&MySimulator::control_callback, this, std::placeholders::_1)
    );
    
    // Inicializamos RViz
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Cargamos el circuito
    track_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/track_markers", 10);

    // Percepción de los conos + ruido
    perception_pub_ = this->create_publisher<my_simulator::msg::ConeArray>("/perception", 10);
    perception_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception_markers", 10);
    noise_dist_ = std::normal_distribution<double>(0.0, 0.1);

    std::string home_path = std::getenv("HOME");
    std::string csv_path = home_path + "/my_ws/src/Proyecto-Percepcion-Arus/my_simulator/tracks/circuit.csv";
    load_circuit(csv_path);

    // Inicializamos timer
    timer_ = this->create_wall_timer(
      100ms, [this](){
        update_physics();
        update_perception(); // <--- Llamamos al sensor en cada ciclo
      });

    // Mensaje de "arranque"
    RCLCPP_INFO(this->get_logger(), "¡Simulador MySimulator iniciado correctamente!");
}

// Lectura del CSV
void MySimulator::load_circuit(std::string path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "NO SE PUDO ABRIR EL ARCHIVO: %s", path.c_str());
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string segment;
    std::vector<std::string> row;

    // Separar por comas
    while (std::getline(ss, segment, ',')) {
      row.push_back(segment);
    }

    if (row.size() >= 3) {
      my_simulator::msg::Cone cone;
      cone.x = std::stof(row[0]);
      cone.y = std::stof(row[1]);
      std::string color = row[2];
      color.erase(0, color.find_first_not_of(" \t")); 
      cone.color = color;
            
      track_cones_.push_back(cone);
    }
  }
  file.close();
  RCLCPP_INFO(this->get_logger(), "Cargados %zu conos.", track_cones_.size());
}

// Guardamos los inputs del ususario
void MySimulator::control_callback(const my_simulator::msg::Control::SharedPtr msg)
{
  acc_ = msg->acc;
  delta_ = msg->delta;
  last_msg_time_ = this->now();
}

// Calcular físicas
void MySimulator::update_physics()
{
  double dt = 0.1;

  // Vemos ultimo input para aplicar friccion
  auto current_time = this->now();
  double time_diff = (current_time - last_msg_time_).seconds();

  if (time_diff > 1.0) {
    acc_ = 0.0;
    delta_ = 0.0; 
  }

  v_ += acc_*dt;

  if (acc_ == 0.0) {
    // Coeficiente de rozamiento
    double friction = 1.0 * dt; 

    if (v_ > 0) {
      v_ -= friction;
      if (v_ < 0) v_ = 0;  // Restamos velocidad hasta llegar a cero
    } 
    else if (v_ < 0) {
      v_ += friction;
      if (v_ > 0) v_ = 0;  // Sumamos velocidad si vamos hacia atras hasta llegar a cero
    }
  }
  
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

  // Visualizamos el circuito
  publish_track();
}

// Percepcion
void MySimulator::update_perception()
{
  my_simulator::msg::ConeArray cone_msg;
  cone_msg.header.stamp = this->now();
  cone_msg.header.frame_id = "base_link";

  visualization_msgs::msg::MarkerArray viz_msg;
  int id_counter = 0;

  for (const auto& global_cone : track_cones_) {
    // Traslación
    double dx = global_cone.x - x_;
    double dy = global_cone.y - y_;

    // Rotación
    double local_x = dx * cos(yaw_) + dy * sin(yaw_);
    double local_y = -dx * sin(yaw_) + dy * cos(yaw_);

    // Filtros del Sensor
    double range = sqrt(local_x*local_x + local_y*local_y);
        
    // Distancia < 20m Y Coordenada X > 0
    if (range < 20.0 && local_x > 0.0) {
            
      // Ruido
      double noise_x = noise_dist_(noise_gen_);
      double noise_y = noise_dist_(noise_gen_);

      my_simulator::msg::Cone perceived_cone;
      perceived_cone.x = local_x + noise_x;
      perceived_cone.y = local_y + noise_y;
      perceived_cone.color = global_cone.color;
            
      cone_msg.cones.push_back(perceived_cone);

      // Visualización para RViz
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "base_link";
      m.header.stamp.sec = 0;
      m.header.stamp.nanosec = 0;
      m.ns = "perception";
      m.id = id_counter++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.lifetime = rclcpp::Duration::from_seconds(0.2);
      m.pose.position.x = perceived_cone.x;
      m.pose.position.y = perceived_cone.y;
      m.pose.position.z = 0.5;

      m.pose.orientation.w = 1.0;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;

      m.scale.x = 0.6; m.scale.y = 0.6; m.scale.z = 1.2;
      m.color.a = 0.5;

      if (perceived_cone.color == "blue") m.color.b = 1.0;
      else if (perceived_cone.color == "yellow") { m.color.r = 1.0; m.color.g = 1.0; }
      else if (perceived_cone.color == "orange" || perceived_cone.color == "orange big") { m.color.r = 1.0; m.color.g = 0.5; }
            
      viz_msg.markers.push_back(m);
    }
  }
    
  // Publicar
  perception_pub_->publish(cone_msg);
  perception_viz_pub_->publish(viz_msg); 
}

void MySimulator::publish_track()
{
  visualization_msgs::msg::MarkerArray markers;
    
  for (size_t i = 0; i < track_cones_.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map"; // Circuito fijo en el mapa
    m.header.stamp = this->get_clock()->now();
    m.ns = "track";
    m.id = i;
    m.type = visualization_msgs::msg::Marker::CYLINDER; // Forma de cono aproximadamente
    m.action = visualization_msgs::msg::Marker::ADD;
        
    m.pose.position.x = track_cones_[i].x;
    m.pose.position.y = track_cones_[i].y;
    m.pose.position.z = 0.5;

    m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 1.0; // Tamaño del cono

    m.color.a = 1.0;
    std::string c = track_cones_[i].color;
        
    // Buscamos si la palabra contiene el color
    if (c.find("blue") != std::string::npos) {
      m.color.b = 1.0f; // Azul
    } else if (c.find("yellow") != std::string::npos) {
        m.color.r = 1.0f; m.color.g = 1.0f; // Amarillo
    } else if (c.find("orange") != std::string::npos) {
        m.color.r = 1.0f; m.color.g = 0.5f; // Naranja
        if (c.find("big") != std::string::npos) { // Naranja++
          m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 1.5;
        }
    } else {
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; // Blanco si falla
    }

    markers.markers.push_back(m);
  }
    
  track_pub_->publish(markers);
}