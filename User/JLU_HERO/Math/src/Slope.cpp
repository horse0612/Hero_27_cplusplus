#include "Slope.hpp"

#include <cmath>

extern "C"
{

void Slope_Init(Slope *slope, const float step, const float deadzone)
{
    slope->target = 0.0f;
    slope->step = step;
    slope->value = 0.0f;
    slope->deadzone = deadzone;
}

void Slope_SetTarget(Slope *slope, const float target)
{
    slope->target = target;
}

void Slope_SetStep(Slope *slope, const float step)
{
    slope->step = step;
}

float Slope_NextVal(Slope *slope)
{
    // 当前值与目标值之间的误差
    const float error = slope->value - slope->target;

    // 误差在死区内，不改变当前值
    if (std::fabs(error) < slope->deadzone)
    {
        return slope->value;
    }

    // 误差小于一个步长，直接到达目标值
    if (std::fabs(error) < slope->step)
    {
        slope->value = slope->target;
    }
    else if (error < 0.0f)
    {
        slope->value += slope->step;
    }
    else if (error > 0.0f)
    {
        slope->value -= slope->step;
    }

    return slope->value;
}

float Slope_GetVal(Slope *slope)
{
    return slope->value;
}
}