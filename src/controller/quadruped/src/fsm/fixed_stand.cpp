#include "quadruped_fsm/state.hpp"
#include "quadruped_common/enum_type.hpp"
#include "iostream"
#include <sstream>
#include <iomanip>
#include <time.h>
#include "stdint.h"
#include <errno.h>

void fixed_stand::State::enter()
{
    LOG_INFO("进入FixedStand模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);

    enter_q_ = hand_->feet_q_;
    enter_foot_pos_ = hand_->feet_pos_;

    // 每次进入状态时重新加载配置文件
    if (config_loader_.loadConfig()) {
        config_loader_.applyConfig(kp_, kd_, phase_t_, target_foot_pos_);
        LOG_INFO("FixedStand参数配置加载成功");
    } else {
        LOG_INFO("配置文件加载失败，使用默认参数");
        // 保留原有的硬编码值作为后备
        for (int i = 0; i < LEG_NUM; i++) {
            kp_.col(i) << 50.0, 30.0, 30.0;
            kd_.col(i) << 0.5, 0.5, 0.5;
            phase_t_(i) = 2.0;
        }
        // 默认足端位置 (相对于hip坐标系, 单位: 米)
        target_foot_pos_.col(0) <<  0.08, 0.0, -0.30;   // 左前
        target_foot_pos_.col(1) <<  0.08, 0.0, -0.30;   // 右前
        target_foot_pos_.col(2) << -0.08, 0.0, -0.30;  // 左后
        target_foot_pos_.col(3) << -0.08, 0.0, -0.30;  // 右后
    }
    
    hand_->motor_cmd_->SetPD(kp_, kd_);
    LOG_INFO("开始FixedStand动作");
    hand_->phase_manager_->ResetPhase(phase_t_);
}
void fixed_stand::State::run()
{
    hand_->phase_manager_->UpdatePhase();
    hand_->feet_pos_ = hand_->calcCurrentWithPhase(enter_foot_pos_,target_foot_pos_,hand_->phase_manager_->phase_);
    hand_->calcFeetQByPos(hand_->feet_pos_);
    hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                               hand_->this_epoch_velocity_, hand_->tauque_);
}
void fixed_stand::State::exit()
{
    LOG_INFO("退出FixedStand模式");
}
