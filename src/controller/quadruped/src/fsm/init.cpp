#include "quadruped_fsm/state.hpp"
#include "quadruped_common/enum_type.hpp"
#include "iostream"
#include <sstream>
#include <iomanip>
#include <time.h>
#include "stdint.h"
#include <errno.h>

void init::State::enter()
{
    LOG_INFO("进入Init模式");
    this->Init();
    hand_->motor_cmd_->SetTorque(z);
    // 原地锁操作
    hand_->inited_ = false;
    hand_->motor_cmd_->MotorInit();
    hand_->motor_cmd_->SetRunMode(10);

    // 加载配置文件
    if (config_loader_.loadConfig()) {
        config_loader_.applyConfig(kp_, kd_, phase_t_, base_pos_);
    } else {
        LOG_INFO("配置文件加载失败，使用默认参数");
        // 使用默认参数
        for (int i = 0; i < LEG_NUM; i++) {
            kp_.col(i) << 10.0, 10.0, 10.0;
            kd_.col(i) << 0.25, 0.25, 0.25;
            phase_t_(i) = 1.0;
        }
        base_pos_.col(0) <<  0.08, 0.0, -0.30;
        base_pos_.col(1) <<  0.08, 0.0, -0.30;
        base_pos_.col(2) << -0.08, 0.0, -0.30;
        base_pos_.col(3) << -0.08, 0.0, -0.30;
    }
    hand_->motor_cmd_->SetPD(kp_, kd_);
}
void init::State::run()
{
    static int wait_log_count = 0;
    if (!hand_->inited_ && hand_->received_state_ && hand_->start_init_)
    {
        // hand_->UpdateInitQ();
        valuable_change_ = true;
        hand_->inited_ = true;
        wait_log_count = 0;
        LOG_INFO("开始初始化");
    }
    else if (!hand_->inited_)
    {
        ++wait_log_count;
        if (wait_log_count >= 1000)
        {
            LOG_INFO("Init等待: joint_state=%d, f_start=%d",
                     hand_->received_state_ ? 1 : 0,
                     hand_->start_init_ ? 1 : 0);
            wait_log_count = 0;
        }
    }
    else if (hand_->inited_ && !stand_)
    {
        if (!stand_prepare_)
        {
            enter_q_ = hand_->current_position_;
            for (int i = 0; i < LEG_NUM; i++)
            {
                hand_->foot_q_ = hand_->quadruped_leg_[i]->calcQ(base_pos_.col(i), FrameType::HIP);
                target_q_.col(i) = hand_->foot_q_;
            }
            stand_prepare_ = true;
            hand_->phase_manager_->ResetPhase(phase_t_);
        }
        else
        {
            if (hand_->phase_manager_->Finished())
            {
                LOG_INFO("初始起身完成");
                stand_ = true;
            }
            hand_->phase_manager_->UpdatePhase();
            hand_->feet_q_ = hand_->calcCurrentWithPhase(enter_q_,target_q_,hand_->phase_manager_->phase_);
            hand_->calcFeetPosByQ(hand_->feet_q_);
            hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                               hand_->this_epoch_velocity_, hand_->tauque_);
        }
    }
    else
    {
        hand_->motor_cmd_->CalcTau(hand_->feet_q_, hand_->this_epoch_position_, hand_->feet_dq_, 
                               hand_->this_epoch_velocity_, hand_->tauque_);
    }
}
void init::State::exit()
{
    stand_ = false;
    stand_prepare_ = false;
    valuable_change_ = false;
    hand_->start_init_ = false;
    LOG_INFO("退出初始化模式");
}
