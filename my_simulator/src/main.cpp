#include "rclcpp/rclcpp.hpp"
#include "my_simulator/my_simulator.hpp"

int main(int argc, char * argv[])
{
    // 1. Inicializar ROS 2
    rclcpp::init(argc, argv);

    // 2. Crear el nodo (usamos shared_ptr como es estándar en ROS 2)
    auto node = std::make_shared<MySimulator>();

    // 3. Mantener el nodo corriendo (spin)
    rclcpp::spin(node);

    // 4. Cerrar limpiamente
    rclcpp::shutdown();
    return 0;
}