#ifndef FSM_HPP
#define FSM_HPP
#include "fsm_list.hpp"
class FSM
{
public:
    FSM(Hand *hand) : hand_(hand)
    {
        Init();
    }
    ~FSM() {}
    void Init();
    void Run();
    void CheckSafty();

    bool stateChanged();
    Hand *hand_;
    FSMMode fsm_mode_;
    FSMStateList *state_list_;
};
#endif