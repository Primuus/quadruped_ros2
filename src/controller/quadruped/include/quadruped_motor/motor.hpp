#ifndef MOTOR_HPP
#define MOTOR_HPP
#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_motor/effort.hpp"
#include <rclcpp/rclcpp.hpp>


class MotorCMD
{
public:
    void MotorInit();
    void SetRunMode(int mode);
    void SetQ(Mat3x4D q);
    void SetDq(Mat3x4D dq);
    void CalcTau(const Mat3x4D& q_des, 
                                           const Mat3x4D& q, 
                                           const Mat3x4D& dq_des, 
                                           const Mat3x4D& dq, 
                                           const Mat3x4D& tau_des);
    
    // 新增力矩控制方法
    void SetTorque(Mat3x4D tau);
    void SetPDGains(double Kp, double Kd);
    void SetPD(Mat3x4D Kp, Mat3x4D Kd);
    
    MotorSetting *motors_set_;
    quadruped_motor::EffortControl* effort_control_;  // 力矩控制器
};
#endif