#pragma once

#include <cstdint>

#include "JLU_HERO/Math/inc/PID.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
inline constexpr float Encoder2Deg = 360.0f / 8192.0f;
struct Gimbal
{
    DMmotor dm8518_yaw{};
    DJImotor dji6020_pitch{};
    struct
    {
        float initAngle{};
        float angle{};
        float gyro{};
        float targetAngle{};
        float Motor_Angle{};
        float Motor_TargetAngle{};
        float MotorAngleOffset{};
        float lastAngle{};
        bool MotorAngleMapFlag_yaw{};
        CascadePID imuPID{};
        PID singleanglePID{};
    } yaw_up;
    struct
    {
        float initAngle{};
        float angle{};
        float lastAngle{};
        float gyro{};
        float targetAngle{};
        float pitchMax{};
        float pitchMin{};
        CascadePID imuPID{};
        PID singleanglePID{};
    } pitch;
    std::int16_t turn_flag{};
    bool yawFeedbackValid{};
    bool pitchFeedbackValid{};
    bool visionEnable{};
    bool rockerCtrl{};
};

extern "C"
{
extern Gimbal gimbal;
extern float pitch_forward;
void Gimbal_Init(void);
void Gimbal_InitPID(void);
void Gimbal_Update(void);
void OS_GimbalCallback(void const *argument);
}
