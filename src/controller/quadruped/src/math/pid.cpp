#include "quadruped_math/pid.hpp"
using namespace math;
Pid::Pid(void)
{
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    p_out_ = 0;
    i_out_ = 0;
    d_out_ = 0;
    measure_value_ = 0;
    target_value_ = 0;
    error_ = 0;
    last_error_ = 0;
    llast_error_ = 0;
    dead_zone_ = 0;
    max_out_ = 0;
    integral_limit_ = 0;
    out_ = 0;
    last_out_ = 0;
}
void Pid::ChangeParam(float max_out, float integral_limit, float kp, float ki, float kd)
{
    max_out_ = max_out;
    integral_limit_ = integral_limit;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float Pid::Calc(float measure, float target)
{
    target_value_ = target;
    error_ = target_value_ - measure;
    if (Abs<float>(error_) < dead_zone_)
    {
        error_ = 0;
    }
    p_out_ = kp_ * error_;
    i_out_ += ki_ * error_;
    d_out_ = kd_ * (error_ - last_error_);
    Limit<float>(&(i_out_), integral_limit_);
    out_ = p_out_ + i_out_ + d_out_;
    Limit<float>(&(out_), max_out_);
    last_out_ = out_;
    llast_error_ = last_error_;
    last_error_ = error_;
    return out_;
}

void Pid::ChangePID(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::SetTarget(float target_value)
{
    target_value_ = target_value;
}

void Pid::ChangeMaxOut(float max_out)
{
    max_out_ = max_out;
}

void Pid::Reset()
{
    p_out_ = 0;
    i_out_ = 0;
    d_out_ = 0;
    measure_value_ = 0;
    target_value_ = 0;
    error_ = 0;
    last_error_ = 0;
    llast_error_ = 0;
    out_ = 0;
    last_out_ = 0;
}