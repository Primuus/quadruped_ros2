#include "quadruped_motor/motor.hpp"
#include "rclcpp/rclcpp.hpp"

void MotorCMD::MotorInit()
{
    // 初始化电机设置
    motors_set_ = new MotorSetting[12];
    for (int i = 0; i < 12; i++) {
        motors_set_[i].mode_ = 10;
        motors_set_[i].q_ = 0.0;
        motors_set_[i].dq_ = 0.0;
        motors_set_[i].tau_ = 0.0;
        motors_set_[i].Kp_ = 40.0;
        motors_set_[i].Kd_ = 0.25;
    }
    
    // 初始化力矩控制器
    effort_control_ = new quadruped_motor::EffortControl();
    effort_control_->Init(motors_set_);
}
void MotorCMD::SetRunMode(int mode)
{
    if (motors_set_ == nullptr) return;
    for (int i = 0; i < 12; i++)
    {
        motors_set_[i].mode_ = mode;
    }
}

void MotorCMD::SetPDGains(double Kp, double Kd)
{
    if (motors_set_ == nullptr) return;
    for (int i = 0; i < 12; i++)
    {
        motors_set_[i].Kp_ = Kp;
        motors_set_[i].Kd_ = Kd;
    }
}

void MotorCMD::SetPD(Mat3x4D Kp, Mat3x4D Kd)
{
    if (motors_set_ == nullptr) return;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            motors_set_[i * 3 + j].Kp_ = Kp(j, i);
            motors_set_[i * 3 + j].Kd_ = Kd(j, i);
        }
    }
}



void MotorCMD::SetQ(Mat3x4D q)
{
    if (motors_set_ == nullptr) return;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            motors_set_[i * 3 + j].q_ = q(j, i);
        }
    }
}

void MotorCMD::SetDq(Mat3x4D dq)
{
    if (motors_set_ == nullptr) return;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            motors_set_[i * 3 + j].dq_ = dq(j, i);
        }
    }
}


void MotorCMD::CalcTau(const Mat3x4D& q_des, 
                                           const Mat3x4D& q, 
                                           const Mat3x4D& dq_des, 
                                           const Mat3x4D& dq, 
                                           const Mat3x4D& tau_des)
{
    effort_control_->ConvertPositionToTorque(q_des, q, dq_des, dq, tau_des);
}

void MotorCMD::SetTorque(Mat3x4D tau)
{
    if (motors_set_ == nullptr) return;  // 防止未初始化时崩溃
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            motors_set_[i * 3 + j].tau_ = tau(j, i);
        }
    }
}


