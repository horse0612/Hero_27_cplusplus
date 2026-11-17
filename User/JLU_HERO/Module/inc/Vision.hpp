#pragma once

#include <cstdint>
#include "ins_task.hpp"
#include "cdc_uart.hpp"

namespace LibXR::USB { class DeviceCore; }

inline constexpr std::uint8_t kVisionTxHeader = 0x5A;
inline constexpr std::uint8_t kVisionTxFinish = 0x6A;
inline constexpr std::uint8_t kVisionRxHeader = 0xA5;
inline constexpr std::uint8_t kVisionRxFinish = 0xA6;

#pragma pack(push, 1)
struct VisionReceive
{
    std::uint8_t header{};
    float yaw{};
    float pitch{};
    std::uint8_t fire{};
    std::uint8_t looking_in : 1;
    std::uint8_t finish{};
    std::uint16_t checksum{};
};

struct Vision_Transmit
{
    std::uint8_t header{};
    float roll{};
    float pitch{};
    float yaw{};
    std::uint8_t finish{};
    std::uint16_t checksum{};
};

struct Vision_Gimbal
{
    float yaw{};
    float pitch{};
    std::uint8_t fire{};
    std::uint8_t found : 1;
};
#pragma pack(pop)

extern "C"
{
extern VisionReceive vision_receive;
extern Vision_Transmit vision_transmit;
extern Vision_Gimbal vision;
void Vision_Init(void);
void Vision_Run(void);
void OS_VisionCallback(void const *argument);
}

void Vision_BindUsb(LibXR::USB::CDCUart& cdc, LibXR::USB::DeviceCore& usb);
