#include "quadruped_fsm/state.hpp"
void deinit::State::enter()
{
    LOG_INFO("进入DeInit模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);
    hand_->motor_cmd_->SetTorque(z);
    hand_->motor_cmd_->SetRunMode(10);
    kp_.col(0) << 0.0, 0.0, 0.0;
    kp_.col(1) << 0.0, 0.0, 0.0;
    kp_.col(2) << 0.0, 0.0, 0.0;
    kp_.col(3) << 0.0, 0.0, 0.0;
    kd_.col(0) << 3.0, 3.0, 3.0;
    kd_.col(1) << 3.0, 3.0, 3.0;
    kd_.col(2) << 3.0, 3.0, 3.0;
    kd_.col(3) << 3.0, 3.0, 3.0;
    Mat3x4D z;
    z.col(0) << 0.0, 0.0, 0.0;
    z.col(1) << 0.0, 0.0, 0.0;
    z.col(2) << 0.0, 0.0, 0.0;
    z.col(3) << 0.0, 0.0, 0.0;
    hand_->motor_cmd_->SetPD(kp_,kd_);
    hand_->motor_cmd_->SetTorque(z);
    hand_->motor_cmd_->SetDq(z);
    hand_->motor_cmd_->SetQ(z);
    LOG_INFO("开始Deinit动作");
}
void deinit::State::run()
{
    // hand_->showTau();
    hand_->SetFeetQ(hand_->current_position_);
    hand_->calcFeetPosByQ(hand_->feet_q_);
    hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, hand_->this_epoch_velocity_, hand_->tauque_);
    // hand_->inited_ = false;
}
void deinit::State::exit()
{
    LOG_INFO("退出DeInit模式");
}
