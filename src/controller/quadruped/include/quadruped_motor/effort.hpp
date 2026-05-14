#ifndef QUADRUPED_MOTOR_EFFORT_HPP_
#define QUADRUPED_MOTOR_EFFORT_HPP_

#include "quadruped_common/eigen_type.hpp"
#include "quadruped_motor/motor_message.hpp"


namespace quadruped_motor {

class EffortControl {
public:
    EffortControl();
    ~EffortControl() = default;
    
    void Init(MotorSetting* motors_set);
    void ConvertPositionToTorque(const Mat3x4D& q_des, const Mat3x4D& q, const Mat3x4D& dq_des, const Mat3x4D& dq, const Mat3x4D& tau_des);
    void SetTorqueLimits(double min_torque, double max_torque);
    
private:
    MotorSetting* motors_set_;
    Mat3x4D target_torque_;
    double min_torque_;
    double max_torque_;
};

}  // namespace quadruped_motor

#endif  // QUADRUPED_MOTOR_EFFORT_HPP_