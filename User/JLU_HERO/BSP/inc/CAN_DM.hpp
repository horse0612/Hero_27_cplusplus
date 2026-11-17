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

#define DM4310_Trig_StaticTorque 0.1f //测试出来的静摩擦力矩


constexpr float P_MIN = -3.14159f;
constexpr float P_MAX = 3.14159f;

constexpr float V_MIN = -20.0f;
constexpr float V_MAX = 20.0f;

constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 500.0f;

constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 15.0f;

constexpr float T_MIN = -10.0f;
constexpr float T_MAX = 10.0f;

struct DMmotor
{
    std::uint16_t id;
    std::uint16_t state;

    int p_int;
    int v_int;
    int t_int;
    float pos;
    float vel;
    float tor;
    float Tmos;
    float Tcoil;
    float last_pos;
    float total_target_pos;
    float totalangle;
    float target_pos;
    float target_vel;
    float target_tor;
    float forward_motor;
   	float Pos_Gear;//中心供弹拨盘，电机与实际具有齿轮传动比关系
    PID singlePID;
    DEPID dePID;
    CascadePID calcPID;
    std::uint8_t Is_Lost;
    std::uint16_t LostCnt;
}; 

extern "C"
{
extern bool start_dm3410;
extern std::uint8_t Data_Enable[8];
extern std::uint8_t Data_Failure[8];
extern std::uint8_t Data_Save_zero[8];
extern DMmotor dmmotor;
int float_to_uint(float value, float minimum, float maximum, int bits);
float uint_to_float(int value, float minimum, float maximum, int bits);
void DMMotor_StartCalcAngle(DmRxSnapshot *motor,DMmotor *dm_motor);
void dm4310_CalcAngle(DmRxSnapshot *motor,DMmotor *dm_motor);
void PosSpeed_CtrlMotor(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId, float position, float velocity);
void Speed_CtrlMotor(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId, float velocity);
void MIT_CtrlMotor(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId, float position, float velocity, float kp, float kd, float torque);
void DMstrat(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
void DMstop(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
void DMerror(FDCAN_HandleTypeDef *hfdcan, std::uint16_t standardId);
}