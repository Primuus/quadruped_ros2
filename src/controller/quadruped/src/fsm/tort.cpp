#include "quadruped_fsm/state.hpp"
#include "iostream"
#include "quadruped_common/enum_type.hpp"
#include <sstream>
#include <iomanip>


void tort::State::enter()
{
    LOG_INFO("进入Tort模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);
    stage_ = 0;
    enter_q_ = hand_->feet_q_;
    enter_foot_pos_ = hand_->feet_pos_;
    target_foot_pos_.col(0) << 0.0, -0.10, -0.30;
    target_foot_pos_.col(1) << 0.0,  0.10, -0.30;
    target_foot_pos_.col(2) << 0.0, -0.10, -0.30;
    target_foot_pos_.col(3) << 0.0,  0.10, -0.30;

    current_config_type_ = "stand";
    config_loader_.setConfigType(current_config_type_);
    // 每次进入状态时重新加载配置文件
    if (config_loader_.loadConfig()) {
        config_loader_.applyConfig(base_pos_, kp_, kd_, target_tort_param_, 
                                 target_delta_param_, phase_t_);
        // LOG_INFO("TORT参数配置加载成功");
    } else {
        LOG_INFO("配置文件加载失败，使用默认参数");
        // 保留原有的硬编码值作为后备
        for (int i = 0; i < LEG_NUM; i++) {
            kp_.col(i) << 10.0, 10.0, 10.0;
            kd_.col(i) << 0.25, 0.25, 0.25;
            phase_t_(i) = 0.2;
            target_tort_param_.row(i) << 0.3, 1.0;
            target_delta_param_.row(i) << 20.0, 0.0, 0.0, 13.0;
        }
    }
    hand_->motor_cmd_->SetPD(kp_,kd_);
    
    LOG_INFO("开始Tort动作");
}
void tort::State::run()
{    

    if (stage_ == 0)
    {
        hand_->phase_manager_->ResetPhase(phase_t_);
        stage_ = 1;
    }
    else if (stage_ == 1)
    {
        hand_->phase_manager_->UpdatePhase();
        hand_->feet_pos_ = hand_->calcCurrentWithPhase(enter_foot_pos_,target_foot_pos_,hand_->phase_manager_->phase_);
        hand_->calcFeetQByPos(hand_->feet_pos_);
        hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                                hand_->this_epoch_velocity_, hand_->tauque_);
        if (hand_->phase_manager_->Finished())
            {
                LOG_INFO("准备完成");
                stage_ = 2;
            }
    }
    else if (stage_ == 2)
    {
        hand_->UpdateTortParam(target_tort_param_, target_delta_param_);
        CreateTrot(base_pos_, hand_->gait_params_, hand_->phase_manager_, mc_);
        stage_ = 3;
    }
    else if (stage_ == 3)
    {   
        std::string new_config_type = "stand";

        if (hand_->y1_dir == 1){
            new_config_type = "walk";
        }
        else if (hand_->y1_dir == -1){
            new_config_type = "back";
        }
        else if (hand_->x1_dir == -1){
            new_config_type = "right";
        }
        else if (hand_->x1_dir == 1){
            new_config_type = "left";
        }
        if (new_config_type != current_config_type_)
        {
            current_config_type_ = new_config_type;
            config_loader_.setConfigType(current_config_type_);
            if (config_loader_.loadConfig())
            {
                config_loader_.applyConfig(base_pos_, kp_, kd_,
                                        target_tort_param_,
                                        target_delta_param_, phase_t_);
                hand_->UpdateTortParam(target_tort_param_, target_delta_param_);
                hand_->motor_cmd_->SetPD(kp_, kd_);
                LOG_INFO("切换配置: %s", current_config_type_.c_str());
            }
        }

        // LOG_INFO("Tort更新中");
        hand_->feet_pos_ =  mc_->UpdateMotion(hand_->phase_manager_, hand_->gait_params_);
        hand_->calcFeetQByPos(hand_->feet_pos_);
        hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                                   hand_->this_epoch_velocity_, hand_->tauque_);
        // hand_->showFeetPos();
    }
}
void tort::State::exit()
{
    LOG_INFO("退出Tort模式");
}
