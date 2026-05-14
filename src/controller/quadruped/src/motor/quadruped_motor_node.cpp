#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_motor/motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

class QuadrupedMotorNode : public rclcpp::Node
{
public:
    explicit QuadrupedMotorNode(const std::string& mode)
    : Node("quadruped_motor_node"), mode_(mode)
    {
        // 初始化电机控制
        motor_cmd_ = std::make_shared<MotorCMD>();
        motor_cmd_->MotorInit();
        
        RCLCPP_INFO(this->get_logger(), "Quadruped motor node initialized in %s mode", mode_.c_str());
    }

private:
    std::string mode_;
    std::shared_ptr<MotorCMD> motor_cmd_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 从参数获取运行模式
    std::string mode = "sim";  // 默认为仿真模式
    
    auto node = std::make_shared<QuadrupedMotorNode>(mode);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}