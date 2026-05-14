#ifndef MOTION_NODE_H
#define MOTION_NODE_H

#include <vector>
#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"

class Node
{
public:
    // 构造函数
    Node(MotionState state, const Vec3D &start, const Vec3D &delta, double height, double duration)
        : state_(state), duration_(duration), height_(height), start_(start), delta_(delta), next_(nullptr), pre_(nullptr)
    {
    }
    ~Node() = default;

    // 链表操作
    Node *Next() const { return next_; }
    Node *Pre() const { return pre_; }
    void SetNext(Node *node) { next_ = node; }
    void SetPre(Node *node) { pre_ = node; }

    // 获取节点
    MotionState GetState() const { return state_; }

    double GetDuration() const { return duration_; }
    double GetHeight() const { return height_; }
    const Vec3D &GetDelta() const { return delta_; }
    const Vec3D &GetStart() const { return start_; }

    // 设置节点
    void SetState(MotionState state) { state_ = state; }
    void SetDuration(double duration) { duration_ = duration; }
    void SetHeight(double height) { height_ = height; }
    void SetStart(const Vec3D &start) { start_ = start; }
    void SetDelta(const Vec3D &delta) { delta_ = delta; }

    int label_;
    MotionState state_; // 动作状态
    double duration_;    // 持续时间(秒)
    double height_;      // 摆动高度
    Vec3D start_;       // 运动初始
    Vec3D delta_;       // 运动增量
    Node *next_;        // 下一个节点
    Node *pre_;         // 上一个节点
};
#endif