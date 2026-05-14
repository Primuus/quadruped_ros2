#ifndef MOTOR_MESSAGE_HPP
#define MOTOR_MESSAGE_HPP
#include "quadruped_common/enum_type.hpp"

class MotorSetting
{
public:
    MotorSetting() : mode_(0), q_(0.0), dq_(0.0), tau_(0.0), Kp_(0.0), Kd_(0.0) {};
    int mode_; // 模式
    double q_;           // 角度
    double dq_;          // 速度
    double tau_;         // 力矩
    double Kp_;          // p
    double Kd_;          // d
};

#endif // MOTOR_MESSAGE_HPP
