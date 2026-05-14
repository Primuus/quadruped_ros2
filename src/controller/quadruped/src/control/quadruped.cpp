#include "rclcpp/rclcpp.hpp"
#include "ros2_interface.hpp"

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // Create a node for the interface
    auto node = std::make_shared<rclcpp::Node>("quadruped_robot");

    // 创建ROS2接口
    auto ros2_interface = std::make_shared<quadruped::ROS2Interface>();
    
    // 初始化ROS2接口
    if (!ros2_interface->initialize(node)) {
        RCLCPP_ERROR(rclcpp::get_logger("quadruped_robot"), "Failed to initialize ROS2 Interface");
        return -1;
    }
    
    // 启动控制循环
    ros2_interface->start();
    
    // Keep the node spinning
    rclcpp::spin(node);
    
    // Stop the interface when shutting down
    ros2_interface->stop();
    
    rclcpp::shutdown();
    return 0;
}