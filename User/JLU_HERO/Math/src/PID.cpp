#include "JLU_HERO/Math/inc/PID.hpp"

namespace
{
inline float Limit(const float value, const float minimum, const float maximum)
{
    if (value <= minimum)
    {
        return minimum;
    }

    if (value >= maximum)
    {
        return maximum;
    }

    return value;
}

inline float Absolute(const float value)
{
    return value >= 0.0f ? value : -value;
}
} // namespace

extern "C"
{

void PID_Init(PID *pid, const float p, const float i, const float d, const float maxIntegral, const float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    pid->maxIntegral = maxIntegral;
    pid->maxOutput = maxOutput;
    pid->deadzone = 0.0f;
}

void DEPID_Init(DEPID *pid, const float p, const float i, const float d, const float maxIntegral, const float maxOutput, const float gama)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    pid->maxIntegral = maxIntegral;
    pid->maxOutput = maxOutput;
    pid->gama = gama;
}

void PIDRegulation(DEPID *pid, const float reference, const float feedback, float differentiation)
{
    pid->lasterror = pid->error;
    pid->error = reference - feedback;

    // 微分低通滤波
    differentiation = pid->gama * differentiation + (1.0f - pid->gama) * pid->lastPv;

    // 微分项
    pid->output = differentiation * pid->kd;

    // 比例项
    pid->output += pid->error * pid->kp;

    // 积分项
    pid->integral += pid->error * pid->ki;
    pid->integral = Limit(pid->integral, -pid->maxIntegral, pid->maxIntegral);

    pid->output += pid->integral;

    // 输出限幅
    pid->output = Limit(pid->output, -pid->maxOutput, pid->maxOutput);

    pid->lastPv = differentiation;
}

void PID_SingleCalc(PID *pid, const float reference, const float feedback)
{
    pid->lastError = pid->error;

    const float difference = reference - feedback;

    if (Absolute(difference) < pid->deadzone)
    {
        pid->error = 0.0f;
    }
    else if (difference < 0.0f)
    {
        pid->error = difference + pid->deadzone;
    }
    else
    {
        pid->error = difference - pid->deadzone;
    }

    // 微分项
    pid->output = (pid->error - pid->lastError) * pid->kd;

    // 比例项
    pid->output += pid->error * pid->kp;

    // 积分项
    pid->integral += pid->error * pid->ki;
    pid->integral = Limit(pid->integral, -pid->maxIntegral, pid->maxIntegral);

    pid->output += pid->integral;

    // 输出限幅
    pid->output = Limit(pid->output, -pid->maxOutput, pid->maxOutput);
}

void PID_CascadeCalc(CascadePID *pid, const float angleRef, const float angleFdb, const float speedFdb)
{
    // 外环：角度环
    PID_SingleCalc(&pid->outer, angleRef, angleFdb);

    // 内环：速度环
    PID_SingleCalc(&pid->inner, pid->outer.output, speedFdb);

    pid->output = pid->inner.output;
}

void DEPID_CascadeCalc(CascadePID *pid, const float angleRef, const float angleFdb, const float speedFdb)
{
    // 外环：微分先行角度环
    PIDRegulation(&pid->deOuter, angleRef, angleFdb, -speedFdb);

    // 内环：速度环
    PID_SingleCalc(&pid->inner, pid->deOuter.output, speedFdb);

    pid->output = pid->inner.output;
}

void EX_DEPID_CascadeCalc(CascadePID *pid, const float angleRef, const float angleFdb, const float speedFdb)
{
    PIDRegulation(&pid->deOuter, angleRef, angleFdb, -speedFdb);

    pid->deOuter.output = Limit(pid->deOuter.output, -pid->deOuter.maxOutput, pid->deOuter.maxOutput);

    if (Absolute(pid->deOuter.error) > 2.0f)
    {
        pid->deOuter.integral *= 0.2f;
    }

    PID_SingleCalc(&pid->inner, pid->deOuter.output, speedFdb);

    pid->output = pid->inner.output;

    pid->output = Limit(pid->output, -20000.0f, 20000.0f);
}

void PID_Clear(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void DEPID_Clear(DEPID *pid)
{
    pid->error = 0.0f;
    pid->lasterror = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->lastPv = 0.0f;
}

void PID_SetMaxOutput(PID *pid, const float maxOutput)
{
    pid->maxOutput = maxOutput;
}

void PID_SetDeadzone(PID *pid, const float deadzone)
{
    pid->deadzone = deadzone;
}
}
