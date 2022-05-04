#include "rm_util/pid.hpp"

using namespace rm_util;

PID::PID(float kp, float ki, float kd, float dt, float min_output, float max_output)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;

    last_error_ = 0;
    integral_ = 0;

    limit_min_ = min_output;
    limit_max_ = max_output;
}

void PID::set_kp(float kp)
{
    kp_ = kp;
}

void PID::set_ki(float ki)
{
    ki_ = ki;
}

void PID::set_kd(float kd)
{
    kd_ = kd;
}

void PID::set_dt(float dt)
{
    dt_ = dt;
}

float PID::get_kp()
{
    return kp_;
}

float PID::get_ki()
{
    return ki_;
}

float PID::get_kd()
{
    return kd_;
}

float PID::get_dt()
{
    return dt_;
}

float PID::calc(float target, float current, float dt)
{
    if (dt == 0.0f)
        dt = dt_;

    float error = target - current;
    integral_ += error * dt;
    float derivative = (error - last_error_) / dt;
    last_error_ = error;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    if (limit_min_ != 0.0f && output < limit_min_)
        output = limit_min_;
    else if (limit_max_ != 0.0f && output > limit_max_)
        output = limit_max_;

    return output;
}