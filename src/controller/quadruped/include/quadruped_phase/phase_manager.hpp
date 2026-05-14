#ifndef PHASE_MANAGER_HPP
#define PHASE_MANAGER_HPP
#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"
#include <time.h>
#include <stdint.h>

enum class PhaseState
{
    Running,
    Finished
};

class PhaseManager
{
public:
    PhaseManager();
    // 设置参数
    void SetStartTime(int leg_id);
    void SetStartTimeBeforeMs(int leg_id, uint64_t ms_before);
    void SetStartTimeBeforeS(int leg_id, double s_before);
    double GetElapsedTimeMs(int leg_id);
    double GetElapsedTimeS(int leg_id);
    void ResetPhase();
    void ResetPhase(Vec4D phase_t);
    void UpdatePhase();
    void SetPhaseT(Vec4D phase_t);

    bool Finished();

    PhaseState phase_state_[4]; // 当前状态

    Vec4D phase_;

    Vec4D phase_t_; // 该阶段周期
    Vec4D passed_t_;
    timespec start_t_[4]; // 该阶段开始时间
};
#endif