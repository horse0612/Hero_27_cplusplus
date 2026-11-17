#include "Gimbal.hpp"

#include "RC.hpp"
#include "Vision.hpp"
#include "User_CAN.hpp"
#include "detect.hpp"
#include "ins_task.hpp"
#include "libxr.hpp"
#include <cstdint>

namespace
{
constexpr float kDegreesToRadians = LibXR::PI / 180.0f;
float Clamp(const float value, const float minimum, const float maximum)
{
    if (value < minimum) 
    {
        return minimum;
    }
    if (value > maximum) 
    {
        return maximum;
    }
    return value;
}
}

extern "C"
{
Gimbal gimbal __attribute__((section(".dtcm_cpp"), aligned(8))) {};
void Gimbal_InitPID(void)
{
    DEPID_Clear(&gimbal.yaw_up.imuPID.deOuter);
    DEPID_Clear(&gimbal.pitch.imuPID.deOuter);
    PID_Clear(&gimbal.pitch.imuPID.inner);
    DEPID_Init(&gimbal.yaw_up.imuPID.deOuter, -4.5f, -0.01f, -35.0f, 2.0f, 10.0f, 0.45f);
    PID_Init(&gimbal.pitch.imuPID.outer, 500.0f, 0.0f, 0.0f, 4500.0f, 20000.0f);
    PID_Init(&gimbal.pitch.imuPID.inner, -100.0f, 0.0f, 0.0f, 7000.0f, 20000.0f);
}

void Gimbal_Init(void)
{
    gimbal = Gimbal{};
    gimbal.pitch.pitchMax = 45.0f;
    gimbal.pitch.pitchMin = -6.0f;
    gimbal.pitch.targetAngle = 20.0f;
    gimbal.visionEnable = false;
    Gimbal_InitPID();
}

void Gimbal_Update(void)
{
    if (rcInfo.SWB == 1) 
    {
        gimbal.visionEnable = true;
    }
    else if (rcInfo.SWD != 1) 
    {
        gimbal.visionEnable = false;
    }

    if (gimbal.visionEnable && vision.found != 0)
    {
        gimbal.yaw_up.targetAngle = vision.yaw;
        gimbal.pitch.targetAngle = Clamp(vision.pitch, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
    }
    else if(rcInfo.SWB == 2)
    {
        gimbal.yaw_up.targetAngle += static_cast<float>(rcInfo.ch1) * 0.35f / 784.0f;
        //static std::uint32_t cycle = 0;
        //gimbal.pitch.targetAngle = 20.0f + 10.0f * std::sin(2.0f * LibXR::PI * cycle * 1.0f / 2000.0f);
        //cycle = (cycle + 1) % 2000;
        gimbal.pitch.targetAngle = Clamp(gimbal.pitch.targetAngle + static_cast<float>(rcInfo.ch2) * 0.35f / 784.0f, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
    }

    gimbal.pitch.angle = -User_Imu_GetRoll();
    gimbal.pitch.gyro = -User_Imu_Get_ROLL_Gyro();
    gimbal.yaw_up.angle = User_Imu_GetYaw();
    gimbal.yaw_up.gyro = User_Imu_Get_YAW_Gyro();
    const float yaw_error = gimbal.yaw_up.targetAngle - gimbal.yaw_up.angle;
    if (yaw_error > 180.0f) 
    {
        gimbal.yaw_up.targetAngle -= 360.0f;
    }
    else if (yaw_error < -180.0f) 
    {
        gimbal.yaw_up.targetAngle += 360.0f;
    }

    if (gimbal.pitchFeedbackValid)
    {
        PID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle , gimbal.pitch.angle , gimbal.pitch.gyro);
    }

    if (gimbal.yawFeedbackValid)
    {
        gimbal.yaw_up.Motor_Angle = gimbal.dm8518_yaw.totalangle;
        if (!gimbal.yaw_up.MotorAngleMapFlag_yaw)
        {
            gimbal.yaw_up.MotorAngleOffset = gimbal.yaw_up.Motor_Angle - gimbal.yaw_up.angle * kDegreesToRadians;
            gimbal.yaw_up.MotorAngleMapFlag_yaw = true;
        }
        gimbal.yaw_up.Motor_TargetAngle = gimbal.yaw_up.MotorAngleOffset + gimbal.yaw_up.targetAngle * kDegreesToRadians;
        PIDRegulation(&gimbal.yaw_up.imuPID.deOuter, gimbal.yaw_up.Motor_TargetAngle, gimbal.yaw_up.Motor_Angle, -gimbal.dm8518_yaw.vel);
    }
}

void OS_GimbalCallback(void const* argument)
{
    (void)argument;
    while (App_IsInitialized() == 0U) LibXR::Thread::Sleep(1);
    for (;;)
    {
        CAN_RefreshGimbalFeedback();
        Gimbal_Update();
        LibXR::Thread::Sleep(1);
    }
}
}
