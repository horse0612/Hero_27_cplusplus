#pragma once
#include <cstdint>
#include "libxr.hpp"
extern "C"
{
#include "app_main.h"
#include "fdcan.h"
}

extern "C"
{
void CAN_Init(void);
void USER_CAN_Send(FDCAN_HandleTypeDef *hfdcan, std::int16_t standardId, std::uint8_t txData[8]);
}

struct DmRxSnapshot
{
    std::uint16_t id;
    std::uint16_t state;
    float pos;
    float vel;
    float tor;
    float Tmos;
    float Tcoil;
    float total_angle;
    float origin_pos;
    float initial_target;
    float last_pos;
    bool origin_set;
    std::uint32_t sequence;
    std::uint64_t timestamp_us;
};

struct DjiRxSnapshot
{
    std::int16_t angle;
    std::int16_t speed;
    std::int16_t torque;
    std::int8_t temperature;
    std::uint32_t sequence;
    std::uint64_t timestamp_us;
    std::uint8_t num_id;
    std::int32_t total_angle;
};

struct Consumed
{
    std::uint32_t pitch_sequence;
    std::uint32_t yaw_sequence;
    std::uint32_t trigger_sequence;
    std::uint32_t friction_sequence[6];
};



struct CanState
{
    std::uint16_t can1_send_error;
    std::uint16_t can2_send_error;
    std::uint16_t can3_send_error;
    std::uint16_t can1_receive_error;
    std::uint16_t can2_receive_error;
    std::uint16_t can3_receive_error;
};
extern CanState can_state;
void CAN_RefreshGimbalFeedback();
void CAN_RefreshShooterFeedback();
