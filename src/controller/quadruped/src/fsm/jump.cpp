#include "quadruped_fsm/state.hpp"
#include "iostream"
#include "quadruped_common/enum_type.hpp"
#include <sstream>
#include <iomanip>

void jump::State::enter()
{
    LOG_INFO("进入Jump模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);

    LOG_INFO("开始Jump动作");
    // 加载多阶段配置
    if (config_loader_.loadConfig()) {
        stages_ = config_loader_.getStages();
        current_stage_ = 0;
        
        // 应用第一阶段
        applyCurrentStage();
    } else {
        LOG_INFO("配置文件加载失败，使用默认参数");
        // 保留原有的硬编码值作为后备
        for (int i = 0; i < LEG_NUM; i++)
        {
            kp_.col(i) << 10.0, 10.0, 10.0;
            kd_.col(i) << 0.25, 0.25, 0.25;
            base_pos_.col(i) << -5.0f, 15.0f, -30.0f;
            phase_t_(i) = 0.2;
        }
        stages_.clear();
        current_stage_ = -1;
    }
}

void jump::State::run()
{
    // 如果有阶段配置，执行多阶段逻辑
    if (!stages_.empty() && current_stage_ >= 0) {

        if (hand_->phase_manager_->Finished()) {
            current_stage_++;
            if (current_stage_ < static_cast<int>(stages_.size())) 
            {
                applyCurrentStage();
            }
            else 
            {
                // 所有阶段完成
                // LOG_INFO("Jump所有阶段完成");
            }
        }
    }

    hand_->feet_pos_ = mc_->UpdateMotion(hand_->phase_manager_, hand_->gait_params_);
    hand_->calcFeetQByPos(hand_->feet_pos_);
    hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                               hand_->this_epoch_velocity_, hand_->tauque_);
}

void jump::State::exit()
{
    LOG_INFO("退出Jump模式");
}

void jump::State::applyCurrentStage()
{
    if (current_stage_ < 0 || current_stage_ >= static_cast<int>(stages_.size())) 
    {
        LOG_ERROR("Jump阶段无效");
        return;
    }
    
    const auto& stage = stages_[current_stage_];
    
    // 应用当前阶段参数
    base_pos_ = stage.base_pos;
    kp_ = stage.kp;
    kd_ = stage.kd;
    phase_t_ = stage.phase;
    
    hand_->motor_cmd_->SetPD(kp_, kd_);    
    CreateStraight(hand_->phase_manager_, mc_, hand_->feet_pos_, base_pos_, phase_t_);
    
    LOG_INFO("Jump阶段[%d/%d]: %s",
             current_stage_ + 1, (int)stages_.size(), 
             stage.name.c_str());
}
