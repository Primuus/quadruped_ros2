#include "quadruped_motor/effort.hpp"
#include "quadruped_motor/motor.hpp"
#include <algorithm>

namespace quadruped_motor {

EffortControl::EffortControl() : motors_set_(nullptr), min_torque_(-20.0), max_torque_(20.0) {
    target_torque_.setZero();
}

void EffortControl::Init(MotorSetting* motors_set) {
    motors_set_ = motors_set;
}

void EffortControl::ConvertPositionToTorque(const Mat3x4D& q_des, 
                                           const Mat3x4D& q, 
                                           const Mat3x4D& dq_des, 
                                           const Mat3x4D& dq, 
                                           const Mat3x4D& tau_des) {
    if (motors_set_ == nullptr) return;
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            // 计算位置误差和速度误差
            double position_error = q_des(j, i) - q(j, i);
            double velocity_error = dq_des(j, i) - dq(j, i);
            
            // 获取电机特定的PD参数
            double Kp = motors_set_[i * 3 + j].Kp_;
            double Kd = motors_set_[i * 3 + j].Kd_;
            
            // 计算力矩: effort = kp*(q_des - q) + kd*(dq_des - dq) + tau_des
            double effort = Kp * position_error + Kd * velocity_error + tau_des(j, i);
            
            // 限制力矩在合理范围内
            // effort = std::max(min_torque_, std::min(max_torque_, effort));
            
            motors_set_[i * 3 + j].tau_ = effort;
        }
    }
}

void EffortControl::SetTorqueLimits(double min_torque, double max_torque) {
    min_torque_ = min_torque;
    max_torque_ = max_torque;
}

}  // namespace quadruped_motor