#include "quadruped_fsm/state.hpp"
#include "quadruped_common/enum_type.hpp"
#include "iostream"
#include <sstream>
#include <iomanip>
#include <time.h>
#include "stdint.h"
#include <errno.h>

void free_angle::State::enter()
{
    LOG_INFO("进入FreeAngle模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);

    enter_q_ = hand_->feet_q_;

    // 每次进入状态时重新加载配置文件
    if (config_loader_.loadConfig()) {
        config_loader_.applyConfig(kp_, kd_, phase_t_, target_q_);
        // LOG_INFO("FreeAngle参数配置加载成功");
    } else {
        LOG_INFO("配置文件加载失败，使用默认参数");
        // 保留原有的硬编码值作为后备
        for (int i = 0; i < LEG_NUM; i++) {
            kp_.col(i) << 10.0, 10.0, 10.0;
            kd_.col(i) << 0.25, 0.25, 0.25;
            phase_t_(i) = 0.2;
            target_q_.col(i) << 0.0, 0.0, 0.0;
        }
    }
    
    hand_->motor_cmd_->SetPD(kp_,kd_);
    LOG_INFO("开始FreeAngle动作");
    hand_->phase_manager_->ResetPhase(phase_t_);
}
void free_angle::State::run()
{
    hand_->phase_manager_->UpdatePhase();
    hand_->feet_q_ = hand_->calcCurrentWithPhase(enter_q_,target_q_,hand_->phase_manager_->phase_);
    hand_->calcFeetPosByQ(hand_->feet_q_);
    hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, hand_->this_epoch_velocity_, hand_->tauque_);
}
void free_angle::State::exit()
{
    LOG_INFO("退出FreeAngle模式");
}
