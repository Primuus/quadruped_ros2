#include "quadruped_motion/node_sequence.hpp"
NodeSequence::NodeSequence()
    : head_(nullptr), current_(nullptr), is_loop_(true), total_duration_(0.0), node_count_(0) {}

NodeSequence::~NodeSequence()
{
    Clear();
}

// 在最后添加节点
Node *NodeSequence::AddNode(MotionState state, const Vec3D &start, const Vec3D &delta, double height, double duration, int label)
{
    Node *new_node = new Node(state, start, delta, height, duration);

    if (IsEmpty())
    {
        // 空链表，新节点成为头节点并自我循环
        head_ = new_node;
        current_ = new_node;
        new_node->SetNext(new_node);
        new_node->SetPre(new_node);
    }
    else
    {
        // 找到最后一个节点 (头节点的前一个)
        Node *last = head_->Pre();
        new_node->SetNext(head_);
        new_node->SetPre(last);
        last->SetNext(new_node);
        head_->SetPre(new_node);
    }
    new_node->label_ = label;
    total_duration_ += duration;
    node_count_++;

    return new_node;
}

void NodeSequence::InsertAfter(Node *node, MotionState state, const Vec3D &start, const Vec3D &delta, double height, double duration)
{
    if (!node)
        return;
    Node *new_node = new Node(state, start, delta, height, duration);

    // 插入新节点
    new_node->SetNext(node->Next());
    new_node->SetPre(node);
    node->Next()->SetPre(new_node);
    node->SetNext(new_node);

    total_duration_ += duration;
    node_count_++;
}

void NodeSequence::RemoveNode(Node *node)
{
    if (!node || IsEmpty()) // 异常
        return;
    if (node->Next() == node) // 唯一节点
    {
        delete node;
        head_ = nullptr;
        current_ = nullptr;
        return;
    }

    // 修改链接
    node->Pre()->SetNext(node->Next());
    node->Next()->SetPre(node->Pre());

    // 如果是头节点，更新头指针
    if (head_ == node)
    {
        head_ = node->Next();
    }

    // 如果是当前节点，更新当前指针
    if (current_ == node)
    {
        current_ = node->Next();
    }

    total_duration_ -= node->GetDuration();
    node_count_--;
    delete node;
}

void NodeSequence::Clear()
{
    if (IsEmpty())
        return;

    Node *current = head_;
    Node *next = nullptr;

    // 断开循环链表以避免无限循环
    head_->Pre()->SetNext(nullptr);

    // 删除所有节点
    while (current)
    {
        next = current->Next();
        delete current;
        current = next;
    }

    head_ = nullptr;
    current_ = nullptr;
    total_duration_ = 0.0;
    node_count_ = 0;
}

void NodeSequence::MoveToNext()
{
    if (current_ && !IsEmpty())
    {
        current_ = current_->Next();
    }
}

void NodeSequence::MoveToPre()
{
    if (current_ && !IsEmpty())
    {
        current_ = current_->Pre();
    }
}

void NodeSequence::UpdateNewHead()
{
    Node *p = Head();
    SetHead(nullptr);
    while (p)
    {
        if (p->label_ == 0)
        {
            SetHead(p);
        }
        p = p->next_;
        if (p == Head())
        {
            p = nullptr;
        }
    }
}