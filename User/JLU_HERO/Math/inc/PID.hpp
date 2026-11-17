#pragma once

#include <cstdint>

struct PID
{
    float kp;
    float ki;
    float kd;

    float error;
    float lastError;

    float integral;
    float maxIntegral;

    float output;
    float maxOutput;

    float deadzone;
};

struct DEPID
{
    float kp;
    float ki;
    float kd;

    float lasterror;
    float error;
    float output;
    float integral;
    float derivative;
    float lastPv;
    float gama;

    float maxOutput;
    float maxIntegral;
};

struct CascadePID
{
    PID inner;
    PID outer;
    DEPID deOuter;
    float output;
};

#ifdef __cplusplus
extern "C"
{
#endif

void PID_Init(PID *pid, float p, float i, float d, float maxIntegral, float maxOutput);

void DEPID_Init(DEPID *pid, float p, float i, float d, float maxIntegral, float maxOutput, float gama);

void PIDRegulation(DEPID *pid, float reference, float feedback, float differentiation);

void PID_SingleCalc(PID *pid, float reference, float feedback);

void PID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);

void DEPID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);

void EX_DEPID_CascadeCalc(CascadePID *pid, float angleRef, float angleFdb, float speedFdb);

void PID_Clear(PID *pid);

void DEPID_Clear(DEPID *pid);

void PID_SetMaxOutput(PID *pid, float maxOutput);

void PID_SetDeadzone(PID *pid, float deadzone);

#ifdef __cplusplus
}
#endif