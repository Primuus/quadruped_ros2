#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include "quadruped_common/enum_type.hpp"
#include "quadruped_motion/node_sequence.hpp"
#include "quadruped_phase/phase_manager.hpp"
#include "quadruped_math/trajectory.hpp"
#include "quadruped_math/gait_param.hpp"
#include "logger/logger.hpp"
// 动作控制器 - 管理多条腿的动作序列
class MotionController
{
public:
    // 管理
    MotionController()
    {
        for (int i = 0; i < LEG_NUM; i++)
        {
            sequences_[i] = nullptr;
        }
    }
    ~MotionController()
    {
        CleanUpSequences();
    }
    void SetSequence(int leg_index, NodeSequence *sequence);
    NodeSequence *GetSequence(int leg_index) const;
    void Addmotion(int leg_index, MotionState state, Mat3x4D start, Mat3x4D delta, Vec4D height, Vec4D t, int label);

    void ReSet(PhaseManager *pm, Vec4D bias);
    Mat3x4D UpdateMotion(PhaseManager *pm,GaitParams params);

    void CleanUpSequences();
    Mat3x4D CalcTotalEndPos();
    Vec3D CalcFootPos(int leg_id,MotionState state,PhaseManager *pm,
                    const Vec3D &start,const Vec3D &delta,double height);
    void UpdateDeltaPos(int leg_id);
    void UpdateTortParam(const Mat4x2F &tort, const Mat4x4D &delta);

    NodeSequence *sequences_[LEG_NUM]; // 每条腿的动作序列

    // 临时变量
    Vec3D foot_pos_;
    Mat3x4D feet_pos_;
    Vec3D start_;
    Vec3D delta_;
    double height_;
    MotionState state_;
    Node *current_;
    NodeSequence *seq_;
};

void CreateStraight(PhaseManager *pm, MotionController *motion, Mat3x4D start, Mat3x4D end, Vec4D phase_t);
void CreateTrot(Mat3x4D start, GaitParams param,PhaseManager *pm, MotionController *motion);

#endif