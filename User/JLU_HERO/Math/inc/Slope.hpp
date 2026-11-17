#pragma once

struct Slope
{
    float target;
    float step;
    float value;
    float deadzone;
};

extern "C"
{

void Slope_Init(Slope *slope, float step, float deadzone);

void Slope_SetTarget(Slope *slope, float target);

void Slope_SetStep(Slope *slope, float step);

float Slope_NextVal(Slope *slope);

float Slope_GetVal(Slope *slope);
}