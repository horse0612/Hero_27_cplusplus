#pragma once
#include "JLU_HERO/Math/inc/PID.hpp"
#include "User_CAN.hpp"
#include <cstdint>
#include <cstring>
extern "C"
{
#include "main.h"
#include "app_main.h"
#include "fdcan.h"
}

struct DJImotor
{
    std::int16_t angle;
    std::int16_t speed;
    std::int16_t torque;
    std::int8_t temp;
    std::int16_t lastSpeed;
    std::int16_t lastAngle;
    std::int32_t totalAngle;

    std::int16_t targetSpeed;
    std::int32_t targetAngle;

    PID singlePID;
    CascadePID calcPID;

    std::uint8_t Is_Lost;
    std::uint16_t LostCnt;
};

extern "C"
{
void DJIMotor_StartCalcAngle(DJImotor *motor);
void DJIMotor_CalcAngle(DJImotor *motor);
void DJIMotor_Update(DJImotor *motor, std::int16_t angle, std::int16_t speed, std::int16_t torque, std::int8_t temperature);
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef *hfdcan, std::int16_t standardId, std::int16_t iq1, std::int16_t iq2, std::int16_t iq3, std::int16_t iq4);

}
