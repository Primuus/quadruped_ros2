#ifndef ROS2_INTERFACE_HPP
#define ROS2_INTERFACE_HPP

#include <memory>
#include <thread>
#include <mutex>

// 包含解耦后的模块头文件
#include "quadruped_control/main.hpp"
#include "quadruped_motor/motor_message.hpp"
#include "quadruped_control/hand.hpp"
#include "quadruped_fsm/fsm.hpp"

// 包含消息头文件
#include "quadruped/msg/motor_command.hpp"
#include "quadruped/msg/robot_state.hpp"
#include "quadruped/msg/gait_command.hpp"
#include "quadruped/msg/joystick_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace quadruped {

class ROS2Interface {
public:
    ROS2Interface();
    ~ROS2Interface();

    bool initialize(rclcpp::Node::SharedPtr node);
    void start();
    void stop();
    void executorSpin(); // 添加这个方法声明

private:
    void controlLoop();
    void publishRobotState();
    
    void motorCommandCallback(const quadruped::msg::MotorCommand::SharedPtr msg);
    void gaitCommandCallback(const quadruped::msg::GaitCommand::SharedPtr msg);
    void keyboardInputCallback(const std_msgs::msg::String::SharedPtr msg);
    void robotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void modeInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void joystickCommandCallback(const quadruped::msg::JoystickCommand::SharedPtr msg);
    
    // 发布电机控制命令的方法
    void publishMotorCommand();

    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Publisher<quadruped::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_motor_cmd_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    
    rclcpp::Subscription<quadruped::msg::MotorCommand>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<quadruped::msg::GaitCommand>::SharedPtr gait_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kbd_input_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_input_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<quadruped::msg::JoystickCommand>::SharedPtr joystick_cmd_sub_;
    
    std::unique_ptr<std::thread> control_thread_;
    std::unique_ptr<std::thread> executor_thread_;
    
    std::unique_ptr<Main> main_controller_;
    
    bool initialized_;
    bool running_;
    double flag_ = 1.0;
    bool lock_output_mode_ = false;
    bool require_joint_state_ = true;
    char last_keyboard_key_ = '\0';
    char last_joystick_key_ = '\0';
    
    // 摇杆数据存储
    struct JoystickData {
        double x1 = 0.0f;
        double y1 = 0.0f;
        double x2 = 0.0f;
        double y2 = 0.0f;
        std::string key;
        rclcpp::Time last_update;
    } joystick_data_;
    
    // IMU数据存储
    struct ImuData {
        double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
        double gyro[3] = {0.0, 0.0, 0.0};
        double accel[3] = {0.0, 0.0, 0.0};
        rclcpp::Time last_update;
    } imu_data_;
};

}  // namespace quadruped

#endif  // ROS2_INTERFACE_HPP
