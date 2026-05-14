#include "quadruped_motion/motion.hpp"
#include "quadruped_leg/leg.hpp"
#include "quadruped_phase/phase_manager.hpp"
#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_math/gait_param.hpp"

// 创建小跑步态
void CreateTrot(Mat3x4D start, GaitParams param,PhaseManager *pm, MotionController *motion)
{
    motion->CleanUpSequences();
    // 建立序列
    NodeSequence *seq[LEG_NUM];
    for (int i(0); i < LEG_NUM; i++)
    {
        seq[i] = new NodeSequence();
        seq[i]->SetLoop(true);
        motion->SetSequence(i, seq[i]);
    }
    // 匹配参数
    Mat3x4D delta;
    Vec4D swing_time;
    Vec4D stance_time;
    Vec4D height;
    for (int i = 0; i < LEG_NUM; i++)
    {
        delta(0, i) = param.delta_(i, 0);
        delta(1, i) = param.delta_(i, 1);
        delta(2, i) = param.delta_(i, 2);
        swing_time(i) = param.tort_param_(i, 0) * param.tort_param_(i, 1);
        stance_time(i) = param.tort_param_(i, 1) - swing_time(i);
        height(i) = param.delta_(i, 3);
    }
    // 动作
    for (int i(0); i < LEG_NUM; i++)
    {
        motion->Addmotion(i, MotionState::EndStance, start, -delta / 2.0, height, stance_time / 2.0, 0);
        motion->Addmotion(i, MotionState::Swing, start, delta, height, swing_time, 1);
        motion->Addmotion(i, MotionState::PreStance, start, -delta / 2.0, height, stance_time / 2.0, 2);
    }
    pm->phase_t_ = stance_time / 2.0;
    motion->ReSet(pm, Vec4D(0.5, 0.0, 0.0, 0.5));
}