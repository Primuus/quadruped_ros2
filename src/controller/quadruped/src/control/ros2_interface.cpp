#include "ros2_interface.hpp"

// 包含消息头文件
#include "quadruped/msg/motor_command.hpp"
#include "quadruped/msg/robot_state.hpp"
#include "quadruped/msg/joystick_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include <map>
#include <vector>

// 包含解耦后的模块头文件
#include "quadruped_control/main.hpp"

namespace quadruped
{

    ROS2Interface::ROS2Interface() : initialized_(false), running_(false)
    {
        // 初始化主控制器
        main_controller_ = std::make_unique<Main>();
    }

    ROS2Interface::~ROS2Interface()
    {
        stop();
    }

    bool ROS2Interface::initialize(rclcpp::Node::SharedPtr node)
    {
        if (!node)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2Interface"), "无效的ROS2节点指针");
            return false;
        }

        node_ = node;

        try
        {
            // 创建发布者
            robot_state_pub_ = node_->create_publisher<quadruped::msg::RobotState>(
                "/robot/state", 10);

            motor_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/joint_effort_controller/commands", 10);

            real_motor_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/real_joint_effort_controller/commands", 10);
                
            diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", 10);

            // 创建订阅者
            motor_cmd_sub_ = node_->create_subscription<quadruped::msg::MotorCommand>(
                "/motor/command_in", 10,
                std::bind(&ROS2Interface::motorCommandCallback, this, std::placeholders::_1));

            robot_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&ROS2Interface::robotStateCallback, this, std::placeholders::_1));

            // IMU订阅 - 保留姿态数据入口，传统PD状态不依赖它
            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                "/imu_plugin/out", 10,
                std::bind(&ROS2Interface::imuCallback, this, std::placeholders::_1));

            gait_cmd_sub_ = node_->create_subscription<quadruped::msg::GaitCommand>(
                "/gait/command", 10,
                std::bind(&ROS2Interface::gaitCommandCallback, this, std::placeholders::_1));

            kbd_input_sub_ = node_->create_subscription<std_msgs::msg::String>(
                "/kbd_input", 10,
                std::bind(&ROS2Interface::keyboardInputCallback, this, std::placeholders::_1));
            mode_input_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/input/mode", 10,
                std::bind(&ROS2Interface::modeInputCallback, this, std::placeholders::_1));
            
            joystick_cmd_sub_ = node_->create_subscription<quadruped::msg::JoystickCommand>(
                "/joystick/command", 10,
                std::bind(&ROS2Interface::joystickCommandCallback, this, std::placeholders::_1));

            flag_ = node_->declare_parameter<double>("output_mode", 1.0);
            lock_output_mode_ = node_->declare_parameter<bool>("lock_output_mode", false);
            require_joint_state_ = node_->declare_parameter<bool>("require_joint_state", true);
            initialized_ = true;

            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error initializing ROS2 interface: %s", e.what());
            initialized_ = false;
            return false;
        }
    }

    void ROS2Interface::start()
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(node_->get_logger(), "ROS2接口未初始化，无法启动");
            return;
        }
        running_ = true;
        // 启动控制线程
        control_thread_ = std::make_unique<std::thread>(&ROS2Interface::controlLoop, this);
    }

    void ROS2Interface::executorSpin()
    {
        LOG_INFO("ROS2 quadruped_control 启动");
    }

    void ROS2Interface::stop()
    {

        running_ = false;

        // 等待控制线程结束
        if (control_thread_ && control_thread_->joinable())
        {
            control_thread_->join();
            control_thread_.reset();
        }

        executor_thread_.reset();

        main_controller_.reset();
    }

    void ROS2Interface::controlLoop()
    {
        LOG_INFO("线程设置完成");
        main_controller_->initialize();
        
        if (require_joint_state_)
        {
            RCLCPP_INFO(node_->get_logger(), "等待关节状态初始化...");
            int wait_joint_state_count = 0;
            while (running_ && !main_controller_->hand_->received_state_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                ++wait_joint_state_count;
                if (wait_joint_state_count >= 100)
                {
                    RCLCPP_WARN(node_->get_logger(), "仍在等待 /joint_states，控制循环尚未进入 FSM");
                    wait_joint_state_count = 0;
                }
            }
            RCLCPP_INFO(node_->get_logger(), "关节状态初始化完成，开始控制循环");
        }
        else
        {
            main_controller_->hand_->received_state_ = true;
            main_controller_->hand_->UpdateCurrentState();
            RCLCPP_WARN(
                node_->get_logger(),
                "实机力矩输出模式不等待 /joint_states，按本地初始状态进入 FSM");
        }
        
        while (running_)
        {
            main_controller_->tick();

            // 发布电机控制命令
            publishMotorCommand();

            // 控制频率：1ms = 1000Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        LOG_INFO("控制循环线程结束");
    }

    void ROS2Interface::publishRobotState()
    {
        if (!initialized_ || !robot_state_pub_ || !main_controller_)
        {
            return;
        }

        auto msg = std::make_unique<quadruped::msg::RobotState>();
        // TODO: Fill in robot state data from main_controller_
        // 例如:
        // msg->header.stamp = node_->get_clock()->now();
        // msg->base_position.x = main_controller_->getCurrentPosition().x();
        // ...

        robot_state_pub_->publish(std::move(msg));
    }

    void ROS2Interface::motorCommandCallback(const quadruped::msg::MotorCommand::SharedPtr msg)
    {
        // TODO: 处理来自ROS2的电机控制命令
        // 将ROS2消息转换为您的控制逻辑能理解的格式
        RCLCPP_INFO(node_->get_logger(), "收到电机控制命令");
    }

    void ROS2Interface::robotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 处理来自robot_gazebo的关节状态信息
        if (!msg || !main_controller_)
        {
            return;
        }

        // 定义预期的关节名称（与robot_controller一致）
        // unitree约定: leg0=FR, leg1=FL, leg2=RR, leg3=RL
        std::vector<std::string> expected_joint_names = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };
        
        // 创建关节名称到索引的映射
        std::map<std::string, int> joint_index_map;
        for (size_t i = 0; i < msg->name.size(); i++) {
            joint_index_map[msg->name[i]] = i;
        }

        // 根据关节名称映射更新电机状态
        for (int i = 0; i < 12; i++) {
            const std::string& joint_name = expected_joint_names[i];
            
            if (joint_index_map.find(joint_name) != joint_index_map.end()) {
                // 找到匹配的关节，更新状态
                int gazebo_index = joint_index_map[joint_name];
                
                // 添加边界检查
                if (gazebo_index >= 0 && gazebo_index < static_cast<int>(msg->position.size())) {
                    main_controller_->hand_->current_position_(i%3, i/3) = msg->position[gazebo_index];
                    
                    // 如果velocity数组不为空且大小匹配，则更新速度
                    if (msg->velocity.size() == msg->name.size() && gazebo_index < static_cast<int>(msg->velocity.size())) {
                        main_controller_->hand_->current_velocity_(i%3, i/3) = msg->velocity[gazebo_index];
                    } else {
                        main_controller_->hand_->current_velocity_(i%3, i/3) = 0.0f;
                    }
                }
            }
        }
        
        if (!main_controller_->hand_->received_state_)
        {
            main_controller_->hand_->received_state_ = true;
        }
        // 调用UpdateCurrentState以更新当前状态
        main_controller_->hand_->UpdateCurrentState();
    }

    void ROS2Interface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!msg || !main_controller_)
        {
            return;
        }

        // 存储IMU数据
        imu_data_.quaternion[0] = msg->orientation.w;
        imu_data_.quaternion[1] = msg->orientation.x;
        imu_data_.quaternion[2] = msg->orientation.y;
        imu_data_.quaternion[3] = msg->orientation.z;
        
        imu_data_.gyro[0] = msg->angular_velocity.x;
        imu_data_.gyro[1] = msg->angular_velocity.y;
        imu_data_.gyro[2] = msg->angular_velocity.z;
        
        imu_data_.accel[0] = msg->linear_acceleration.x;
        imu_data_.accel[1] = msg->linear_acceleration.y;
        imu_data_.accel[2] = msg->linear_acceleration.z;
        
        imu_data_.last_update = node_->get_clock()->now();

        // 更新Hand中的IMU数据
        main_controller_->hand_->imu_quaternion_[0] = msg->orientation.w;
        main_controller_->hand_->imu_quaternion_[1] = msg->orientation.x;
        main_controller_->hand_->imu_quaternion_[2] = msg->orientation.y;
        main_controller_->hand_->imu_quaternion_[3] = msg->orientation.z;
        
        main_controller_->hand_->imu_gyro_[0] = msg->angular_velocity.x;
        main_controller_->hand_->imu_gyro_[1] = msg->angular_velocity.y;
        main_controller_->hand_->imu_gyro_[2] = msg->angular_velocity.z;
        
        main_controller_->hand_->imu_acc_[0] = msg->linear_acceleration.x;
        main_controller_->hand_->imu_acc_[1] = msg->linear_acceleration.y;
        main_controller_->hand_->imu_acc_[2] = msg->linear_acceleration.z;

        // 计算旋转矩阵 rot_mat_b2g_ (Body到Global)
        double w = msg->orientation.w;
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;
        
        // 四元数到旋转矩阵 (Hamilton 约定)
        main_controller_->hand_->rot_mat_b2g_(0,0) = 1 - 2*(y*y + z*z);
        main_controller_->hand_->rot_mat_b2g_(0,1) = 2*(x*y - w*z);
        main_controller_->hand_->rot_mat_b2g_(0,2) = 2*(x*z + w*y);
        main_controller_->hand_->rot_mat_b2g_(1,0) = 2*(x*y + w*z);
        main_controller_->hand_->rot_mat_b2g_(1,1) = 1 - 2*(x*x + z*z);
        main_controller_->hand_->rot_mat_b2g_(1,2) = 2*(y*z - w*x);
        main_controller_->hand_->rot_mat_b2g_(2,0) = 2*(x*z - w*y);
        main_controller_->hand_->rot_mat_b2g_(2,1) = 2*(y*z + w*x);
        main_controller_->hand_->rot_mat_b2g_(2,2) = 1 - 2*(x*x + y*y);
        
        // 全局加速度 = 旋转矩阵 * 局部加速度
        main_controller_->hand_->acc_global_ = 
            main_controller_->hand_->rot_mat_b2g_ * 
            Eigen::Vector3d(msg->linear_acceleration.x, 
                            msg->linear_acceleration.y, 
                            msg->linear_acceleration.z);
        
        // 全局陀螺仪 = 旋转矩阵 * 局部陀螺仪
        main_controller_->hand_->gyro_global_ = 
            main_controller_->hand_->rot_mat_b2g_ * 
            Eigen::Vector3d(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);
    }

    void ROS2Interface::publishMotorCommand()
    {
        if (!initialized_ || !motor_cmd_pub_ || !main_controller_ || !real_motor_cmd_pub_)
        {
            return;
        }
        if (main_controller_->hand_->inited_)
        {
            // 直接发送Float64MultiArray到robot_gazebo
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();

            MotorSetting *m = main_controller_->getMotorSettings();

            // motors_set_按unitree顺序填充: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
            // Gazebo go1_ros_control.yaml期望顺序: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
            // 顺序一致，无需重排
            msg->data.clear();
            for (int i = 0; i < 12; i++)
            {
                double tau = m[i].tau_;
                if (!std::isfinite(tau)) {
                    tau = 0.0;
                }
                if (std::abs(tau) > 80.0) {
                    tau = std::copysign(80.0, tau);
                }
                msg->data.push_back(tau);
            }
            if (flag_ == 0.0) {
                motor_cmd_pub_->publish(std::move(msg));
            } else if (flag_ == 1.0) {
                real_motor_cmd_pub_->publish(std::move(msg));
            }
        }
    }

    void ROS2Interface::gaitCommandCallback(const quadruped::msg::GaitCommand::SharedPtr msg)
    {
        // TODO: 处理来自ROS2的步态控制命令
        // 将ROS2消息转换为您的控制逻辑能理解的格式
        RCLCPP_INFO(node_->get_logger(), "收到步态控制命令");
    }

    void ROS2Interface::keyboardInputCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        char key = msg->data.empty() ? '\0' : msg->data[0];
        if (key != '\0' && key != last_keyboard_key_)
        {
            RCLCPP_INFO(node_->get_logger(), "收到 /kbd_input 按键: %c", key);
        }
        last_keyboard_key_ = key;
        if (main_controller_)
        {
            main_controller_->handleKeyboardInput(key);
        }
    }
    void ROS2Interface::modeInputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!msg || msg->data.empty()) {
            RCLCPP_WARN(node_->get_logger(), "modeInputCallback got empty msg");
            return;
        }

        if (!lock_output_mode_) {
            if (flag_ != msg->data[0]) {
                RCLCPP_INFO(node_->get_logger(), "输出模式切换为 %.1f", msg->data[0]);
            }
            flag_ = msg->data[0];
        }
    }
    
    void ROS2Interface::joystickCommandCallback(const quadruped::msg::JoystickCommand::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        
        // 存储摇杆数据
        joystick_data_.x1 = msg->x1;
        joystick_data_.y1 = msg->y1;
        joystick_data_.x2 = msg->x2;
        joystick_data_.y2 = msg->y2;
        joystick_data_.key = msg->key;
        joystick_data_.last_update = node_->get_clock()->now();

        const char key = msg->key.empty() ? '\0' : msg->key[0];
        if (key != '\0' && key != last_joystick_key_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "收到 /joystick/command 按键: %c, x1=%.3f, y1=%.3f, x2=%.3f, y2=%.3f",
                key,
                msg->x1,
                msg->y1,
                msg->x2,
                msg->y2);
        }
        last_joystick_key_ = key;
        
        // 将摇杆数据传递给主控制器
        if (main_controller_)
        {
            main_controller_->handleJoystickInput(
                msg->x1, msg->y1, msg->x2, msg->y2, msg->key);
        }
    }
} // namespace quadruped
