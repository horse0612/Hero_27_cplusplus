#pragma once

#include "CAN_DJI.hpp"
#include "xr_hardware.hpp"
#include "libxr.hpp"
#include <cstdint>
#include <cstring>
extern "C"
{
#include "main.h"
#include "app_main.h"
#include "fdcan.h"
}

struct LKmoto
{
    std::uint16_t angle;
    std::int16_t speed;
    std::int16_t voltage;
    std::int16_t current;
    std::int8_t torque;
    std::int8_t errorCode;
    std::int8_t temp;
    std::int8_t motorstage;
    std::int16_t encoderangle;
    std::int16_t LKtargetspeed;
};

extern "C"
{
void STARTRUN(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
void CLOSEMOTO(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
void CLEARERROR(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
void iqControl(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId, std::int16_t iqControlValue);
void Update9025Motor(LKmoto *motor, std::uint8_t *rxData);
}