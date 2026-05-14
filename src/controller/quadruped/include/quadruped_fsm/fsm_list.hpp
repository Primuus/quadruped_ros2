#ifndef FSM_LIST_HPP
#define FSM_LIST_HPP
#include <vector>
#include <array>
#include <stdexcept>
#include "quadruped_fsm/state.hpp"
class FSMStateList
{
public:
    FSMStateList(Hand *hand)
    {
        hand_ = hand;
        state_list_[static_cast<int>(StateName::Invaild)] = new deinit::State(hand);
        state_list_[static_cast<int>(StateName::Init)] = new init::State(hand);
        state_list_[static_cast<int>(StateName::Deinit)] = new deinit::State(hand);
        state_list_[static_cast<int>(StateName::Tort)] = new tort::State(hand);
        state_list_[static_cast<int>(StateName::Jump)] = new jump::State(hand);
        state_list_[static_cast<int>(StateName::FreeAngle)] = new free_angle::State(hand);
        state_list_[static_cast<int>(StateName::FixedStand)] = new fixed_stand::State(hand);
        state_list_[static_cast<int>(StateName::Turn)] = new turn::State(hand);

        current_state_n_ = StateName::Init;
        next_state_n_ = StateName::Invaild;
        current_state_ = getStateByName(current_state_n_);
        change_valid_ = true;
    }

    // 通过名字获取状态
    base::State *getStateByName(StateName name)
    {
        if (name == StateName::Count)
        {
            return nullptr;
        }
        return state_list_[static_cast<size_t>(name)];
    }

    StateName getNextStateName()
    {
        return next_state_n_;
    }

    void setNextState()
    {
        current_state_ = getStateByName(next_state_n_);
        current_state_n_ = next_state_n_;
        next_state_n_ = StateName::Invaild;
    }

    base::State *current_state_;
    StateName current_state_n_;
    StateName next_state_n_;
    bool change_valid_;

private:
    Hand *hand_;
    std::array<base::State *, static_cast<int>(StateName::Count)> state_list_;
};
#endif
