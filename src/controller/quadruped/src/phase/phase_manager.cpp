#include "quadruped_phase/phase_manager.hpp"
#include "quadruped_common/enum_type.hpp"

PhaseManager::PhaseManager()
{
    for (int i(0); i < LEG_NUM; ++i)
    {
        phase_state_[i] = PhaseState::Finished;
    }
}
// 设置初始时间
void PhaseManager::SetStartTime(int leg_id)
{
    clock_gettime(CLOCK_MONOTONIC, &start_t_[leg_id]);
}
// 设置当前时间之前n毫秒的时间为开始时间
void PhaseManager::SetStartTimeBeforeMs(int leg_id, uint64_t ms_before)
{
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    // 计算当前时间减去指定毫秒数后的时间
    uint64_t total_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
    total_ms -= ms_before;

    start_t_[leg_id].tv_sec = total_ms / 1000;
    start_t_[leg_id].tv_nsec = (total_ms % 1000) * 1000000;
}
// 设置当前时间之前n秒的时间为开始时间
void PhaseManager::SetStartTimeBeforeS(int leg_id, double s_before)
{
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    // 计算当前时间减去指定毫秒数后的时间
    uint64_t total_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
    total_ms -= s_before * 1000.0;

    start_t_[leg_id].tv_sec = total_ms / 1000;
    start_t_[leg_id].tv_nsec = (total_ms % 1000) * 1000000;
}
// 计算经过的时间（毫秒）
double PhaseManager::GetElapsedTimeMs(int leg_id)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    double elapsed = (now.tv_sec - start_t_[leg_id].tv_sec) * 1000.0;
    elapsed += (now.tv_nsec - start_t_[leg_id].tv_nsec) / 1000000.0;
    passed_t_[leg_id] = elapsed / 1000.0;
    return elapsed;
}
// 计算经过的时间（秒）
double PhaseManager::GetElapsedTimeS(int leg_id)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    double elapsed = (now.tv_sec - start_t_[leg_id].tv_sec) * 1000.0;
    elapsed += (now.tv_nsec - start_t_[leg_id].tv_nsec) / 1000000.0;
    passed_t_[leg_id] = elapsed / 1000.0;
    return elapsed / 1000.0;
}
// 更新phase
void PhaseManager::UpdatePhase()
{
    for (int i = 0; i < LEG_NUM; i++)
    {
        phase_(i) = GetElapsedTimeS(i) / phase_t_(i);
        if (phase_(i) >= 1.0f)
        {
            phase_(i) = 1.0f;
            phase_state_[i] = PhaseState::Finished;
            continue;
        }
    }
}
void PhaseManager::ResetPhase()
{
    for (int i = 0; i < LEG_NUM; i++)
    {
        phase_(i) = 0;
        SetStartTime(i);
        phase_state_[i] = PhaseState::Running;
    }
}

void PhaseManager::ResetPhase(Vec4D phase_t)
{
    ResetPhase();
    phase_t_ = phase_t;
}
void PhaseManager::SetPhaseT(Vec4D phase_t)
{
    phase_t_ = phase_t;
    UpdatePhase();
}

bool PhaseManager::Finished()
{
    for (int i = 0; i < LEG_NUM; i++)
    {
        if (phase_state_[i] != PhaseState::Finished)
        {
            return false;
        }
    }
    return true;
}