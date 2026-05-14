#include "quadruped_motion/motion.hpp"

void MotionController::CleanUpSequences()
{
    for (int i = 0; i < LEG_NUM; i++)
    {
        if (sequences_[i] != nullptr)
        {
            delete sequences_[i];
            sequences_[i] = nullptr;
        }
    }
}
// 设置动作序列
void MotionController::SetSequence(int leg_index, NodeSequence *sequence)
{
    if (leg_index < 0 || leg_index >= LEG_NUM)
        return;

    if (sequences_[leg_index])
    {
        delete sequences_[leg_index];
    }

    sequences_[leg_index] = sequence;
}

// lr_seq->AddNode(MotionState::EndStance, start.col(0), -delta.col(0) / 2.0f, height[0], stance_time[0] / 2.0f, 0);
void MotionController::Addmotion(int leg_index, MotionState state, Mat3x4D start, Mat3x4D delta, Vec4D height, Vec4D t, int label)
{
    NodeSequence *q = GetSequence(leg_index);
    q->AddNode(state, start.col(leg_index), delta.col(leg_index), height(leg_index), t(leg_index), label);
}
// 获取动作序列
NodeSequence *MotionController::GetSequence(int leg_index) const
{
    if (leg_index < 0 || leg_index >= LEG_NUM)
        return nullptr;
    return sequences_[leg_index];
}

// 重置足端轨迹回到头的位置
void MotionController::ReSet(PhaseManager *pm, Vec4D bias)
{
    pm->ResetPhase();
    for (int i = 0; i < LEG_NUM; i++)
    {
        if (sequences_[i] && !sequences_[i]->IsEmpty()) // 序列有效且有动作
        {
            sequences_[i]->SetCurrent(sequences_[i]->Head()); // 回到最初
        }
        
        // 计算初始偏置时间
        pm->passed_t_(i) = sequences_[i]->GetTotalDuration() * bias(i);
        
        // 找到应该开始的节点并计算剩余时间
        Node *p = sequences_[i]->Head();
        double remaining_time = pm->passed_t_(i);
        
        // 遍历节点直到剩余时间小于当前节点持续时间
        while (remaining_time > p->GetDuration())
        {
            remaining_time -= p->GetDuration();
            p = p->Next();
        }
        
        // 设置相位管理器的时间和当前节点
        pm->SetStartTimeBeforeS(i, remaining_time);
        sequences_[i]->SetCurrent(p);
        
        // 重要：设置当前阶段的总时间，对于trot步态，对角线腿应有相同的phase_t_
        pm->phase_t_(i) = p->GetDuration();  // 设置当前阶段的总时间
        pm->phase_(i) = static_cast<double>(remaining_time / p->GetDuration());  // 设置当前相位比例
        
        // 如果计算出的相位超过1，则将其限制在0-1之间
        if (pm->phase_(i) > 1.0) {
            pm->phase_(i) = 1.0;
        }
    }
}
Mat3x4D MotionController::CalcTotalEndPos()
{
    Mat3x4D result;
    result.setZero();
    Node *p = nullptr;
    for (int i = 0; i < LEG_NUM; i++)
    {
        p = sequences_[i]->Head();
        while (p)
        {
            result.col(i) += p->delta_;
            p = p->next_;
            if (p == sequences_[i]->Head())
            {
                p = nullptr;
            }
        }
    }
    return result;
}

Vec3D MotionController::CalcFootPos(int leg_id,
                                    MotionState state,
                                    PhaseManager *pm,
                                    const Vec3D &start,
                                    const Vec3D &delta,
                                    double height)
{
    Vec3D foot_pos;
    double phase = pm->phase_(leg_id);
    if (phase >= 1.0)
    {
        phase = 1.0;
    }
    switch (state)
    {
    case MotionState::Swing:
        foot_pos = MoveLegWithSwingHeightIncrement(start, delta, height, phase);
        break;
    case MotionState::Stance:
    case MotionState::PreStance:
    case MotionState::EndStance:
        foot_pos = MoveLegLinearIncrement(start, delta, height, phase);
        // foot_pos = MoveLegWithLandHeightIncrement(start, delta, -1.0, phase);
        break;
    case MotionState::Angle:
        foot_pos = MoveLegAngleIncrement(start, delta, height, phase);
        break;
    case MotionState::TransSwing:
        foot_pos = MoveLegWithSwingHeightIncrement(start, delta, height, phase);
        break;
    case MotionState::TransStance:
        foot_pos = MoveLegLinearIncrement(start, delta, height, phase);
        break;
    case MotionState::Step:
        foot_pos = MoveLegWithStepHeightIncrement(start, delta, height, phase);
        break;
    case MotionState::Stop:
    default:
        foot_pos = start;
        break;
    }
    return foot_pos;
}
// 调整delta后更新startpos

void MotionController::UpdateDeltaPos(int leg_id)
{
    NodeSequence *seq = GetSequence(leg_id);
    if (!seq->IsEmpty())
    {
        if (seq->IsLoop())
        {
            Node *p = seq->Head();
            int head_flag = 2;
            while (head_flag)
            {
                if (p->next_->label_ != 0)
                {
                    p->next_->start_ = p->start_ + p->delta_; // 更新下一个节点的起始位置
                }
                else
                {
                    head_flag--;
                    p->delta_ = p->next_->start_ - p->start_; // 更新当前节点的位移
                }
                p = p->next_;
            }
        }
        else
        {
            Node *p = seq->Head();
            while (p->next_->label_ != 0)
            {
                p->next_->start_ = p->start_ + p->delta_; // 更新下一个节点的起始位置
                p = p->next_;
            }
        }
    }
}

void MotionController::UpdateTortParam(const Mat4x2F &tort, const Mat4x4D &delta)
{
    for (int leg_id = 0; leg_id < LEG_NUM; leg_id++)
    {
        NodeSequence *seq = GetSequence(leg_id);
        if (!seq->IsEmpty())
        {
            if (seq->IsLoop())
            {
                Node *p = seq->Head();
                p->duration_ = tort(leg_id, 1) * (1 - tort(leg_id, 0));
                p->delta_(0) = delta(leg_id, 0) * (-0.5);
                p->delta_(1) = delta(leg_id, 1) * (-0.5);
                p->delta_(2) = delta(leg_id, 2) * (-0.5);
                p = p->next_;
                while(p->label_!=0)
                {
                    if (p->label_ == 1)
                    {
                        p->duration_ = tort(leg_id, 1) * tort(leg_id, 0);
                        p->delta_(0) = delta(leg_id, 0);
                        p->delta_(1) = delta(leg_id, 1);
                        p->delta_(2) = delta(leg_id, 2);
                        p->height_ = delta(leg_id, 3);
                    }
                    else if (p->label_ == 2)
                    {
                        p->duration_ = tort(leg_id, 1) * (1 - tort(leg_id, 0));
                        p->delta_(0) = delta(leg_id, 0) * (-0.5);
                        p->delta_(1) = delta(leg_id, 1) * (-0.5);
                        p->delta_(2) = delta(leg_id, 2) * (-0.5);
                    }
                    p = p->next_;
                }
                
            }
        }
    }
}

Mat3x4D MotionController::UpdateMotion(PhaseManager *pm, GaitParams params)
{
    for (int i = 0; i < LEG_NUM; i++)
    {
        seq_ = GetSequence(i);
        if (!seq_->IsEmpty()) // 有动作
        {
            if (pm->phase_state_[i] == PhaseState::Finished)
            {
                if (seq_->IsLoop())
                {
                    seq_->MoveToNext();
                    // 只重置当前腿的相位，而不是全部腿
                    pm->phase_(i) = 0;
                    pm->passed_t_(i) = 0;
                    pm->SetStartTime(i);
                    pm->phase_state_[i] = PhaseState::Running;
                    pm->phase_t_(i) = seq_->Current()->GetDuration();
                }
                else
                {
                    if (seq_->Current()->next_ != seq_->Head())
                    {
                        seq_->MoveToNext();
                        // 只重置当前腿的相位，而不是全部腿
                        pm->phase_(i) = 0;
                        pm->passed_t_(i) = 0;
                        pm->SetStartTime(i);
                        pm->phase_state_[i] = PhaseState::Running;
                        pm->phase_t_(i) = seq_->Current()->GetDuration();
                    }
                }
            }
            if (seq_->IsLoop())
            {
                UpdateTortParam(params.tort_param_, params.delta_);
                UpdateDeltaPos(i);
            }
            pm->UpdatePhase();
            current_ = seq_->Current();
            state_ = current_->GetState();
            start_ = current_->GetStart();
            delta_ = current_->GetDelta();
            height_ = current_->GetHeight();
            foot_pos_ = CalcFootPos(i, state_, pm, start_, delta_, height_);

        }
        else
        {
            foot_pos_ = feet_pos_.col(i);
        }
        feet_pos_.col(i) = foot_pos_;
    }
    return feet_pos_;
}
