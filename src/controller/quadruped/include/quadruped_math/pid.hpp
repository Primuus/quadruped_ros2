#ifndef PID_HPP
#define PID_HPP
#include "stdint.h"
#include "quadruped_math/math_ant.hpp"

#ifdef __cplusplus

class PidSet
{
public:
    float kp_, ki_, kd_;   // 比例、积分、微分系数
    float max_out_;        // 最大输出限制
    float dead_zone_;      // 死区
    float integral_limit_; // 积分限幅

    PidSet &operator=(const PidSet &set)
    {
        kp_ = set.kp_;
        ki_ = set.ki_;
        kd_ = set.kd_;
        max_out_ = set.max_out_;
        dead_zone_ = set.dead_zone_;
        integral_limit_ = set.integral_limit_;
        return *this;
    }
};

class Pid
{
public:
    float kp_, ki_, kd_;   // 比例、积分、微分系数
    float dead_zone_;      // 死区
    float max_out_;        // 最大输出限制
    float integral_limit_; // 积分限幅
    float target_value_;   // 目标值
    float out_;            // 输出值
    float last_out_;       // 上一次输出值

    Pid();
    void ChangeParam(float max_out, float integral_limit, float kp, float ki, float kd);
    float Calc(float measure, float target);      // 计算PID输出
    void ChangePID(float kp, float ki, float kd); // 修改PID参数
    void SetTarget(float target_value);           // 设置目标值
    void ChangeMaxOut(float max_out);             // 修改最大输出限制
    void Reset();
    Pid &operator=(const PidSet &set)
    {
        kp_ = set.kp_;
        ki_ = set.ki_;
        kd_ = set.kd_;
        dead_zone_ = set.dead_zone_;
        max_out_ = set.max_out_;
        integral_limit_ = set.integral_limit_;
        return *this;
    }

private:
    float p_out_, i_out_, d_out_; // 比例、积分、微分输出
    float measure_value_;         // 测量值
    float error_;                 // 当前误差
    float last_error_;            // 上一次误差
    float llast_error_;           // 上上次误差
};

#endif // __cplusplus
#endif