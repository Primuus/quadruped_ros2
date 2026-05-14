#ifndef NODE_SEQUENCE_HPP
#define NODE_SEQUENCE_HPP

#include "quadruped_motion/node.hpp"

// 动作序列 - 管理循环链表
class NodeSequence
{
public:
    NodeSequence();
    ~NodeSequence();

    // 链表操作
    Node *AddNode(MotionState state, const Vec3D &start, const Vec3D &delta, double height, double peroid, int label);
    void InsertAfter(Node *node, MotionState state, const Vec3D &start, const Vec3D &delta, double height, double peroid);
    void RemoveNode(Node *node);
    void Clear();

    // 序列访问和操作s
    Node *Head() const { return head_; }
    Node *Current() const { return current_; }
    void SetCurrent(Node *node) { current_ = node; }
    void MoveToNext();
    void MoveToPre();
    bool IsEmpty() const { return head_ == nullptr; }

    // 序列属性
    bool IsLoop() const { return is_loop_; }
    void SetLoop(bool loop) { is_loop_ = loop; }
    double GetTotalDuration() const { return total_duration_; }
    int GetNodeCount() const { return node_count_; }

    void SetHead(Node *head) { head_ = head; }

    void UpdateNewHead();

private:
    Node *head_;           // 头节点
    Node *current_;        // 当前节点
    bool is_loop_;         // 是否循环执行
    double total_duration_; // 总周期
    int node_count_;       // 节点数量
};

#endif
