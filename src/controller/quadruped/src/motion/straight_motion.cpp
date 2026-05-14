#include "quadruped_motion/motion.hpp"
#include "quadruped_phase/phase_manager.hpp"
void CreateStraight(PhaseManager *pm, MotionController *motion, Mat3x4D start, Mat3x4D end, Vec4D phase_t)
{
    motion->CleanUpSequences();
    // 建立序列
    NodeSequence *seq[LEG_NUM];
    for (int i(0); i < LEG_NUM; i++)
    {
        seq[i] = new NodeSequence();
        seq[i]->SetLoop(false);
        motion->SetSequence(i, seq[i]);
    }
    // 动作
    for (int i(0); i < LEG_NUM; i++)
    {
        motion->Addmotion(i, MotionState::Stance, start, end - start, Vec4D(0, 0, 0, 0), phase_t, 0);
    }

    motion->ReSet(pm, Vec4D(0, 0, 0, 0));
    pm->phase_t_ = phase_t;
}