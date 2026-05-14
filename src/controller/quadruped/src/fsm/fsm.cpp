#include "quadruped_fsm/fsm.hpp"
#include "quadruped_common/enum_type.hpp"
#include "logger/logger.hpp"

void FSM::Init()
{
    state_list_ = new FSMStateList(hand_);
    fsm_mode_ = FSMMode::NORMAL;
}

void FSM::Run()
{
    if (fsm_mode_ == FSMMode::NORMAL)
    {
        hand_->updateKbdCmd();
        hand_->UpdateCurrentState();
        state_list_->next_state_n_ = hand_->next_state_n_;
        state_list_->current_state_->run();
        if (stateChanged())
        {
            if (state_list_->change_valid_)
            {
                fsm_mode_ = FSMMode::CHANGE;
            }
        }
        else if (hand_->update_param_)
        {
            fsm_mode_ = FSMMode::CHANGE;
        }
    }
    if (fsm_mode_ == FSMMode::CHANGE)
    {
        state_list_->current_state_->exit();
        LOG_INFO("状态机更新");
        if (hand_->update_param_)
        {
            hand_->update_param_ = false;
        }
        else
        {
            state_list_->setNextState();
        }
        state_list_->current_state_->enter();
        fsm_mode_ = FSMMode::NORMAL;
    }
}

bool FSM::stateChanged()
{
    if (state_list_->current_state_n_ != state_list_->next_state_n_)
    {
        return state_list_->next_state_n_ != StateName::Invaild;
    }
    return false;
}
void FSM::CheckSafty()
{
}
