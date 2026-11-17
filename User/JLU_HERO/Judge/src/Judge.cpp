#include "Judge.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "operation.hpp"
#include "semaphore.hpp"
#include "uart.hpp"
#include "xr_hardware.hpp"
#include <algorithm>
#include <cstring>
extern "C"
{
#include "cmsis_os.h"
}
JUDGE_Link judge;
JUDGE_Link::JUDGE_Link():judge_uart(nullptr),judge_wait_callback(),judge_wait(),frame_buffer_{},frame_size_(0) {}

extern "C"
{
ext_game_status_t GameState{};
ext_game_robot_HP_t GameRobotHP{};
ext_event_data_t EventData{};
ext_game_robot_status_t GameRobotState{};
ext_power_heat_data_t PowerHeatData{};
ext_game_robot_pos_t GameRobotPos{};
ext_buff_musk_t BuffMusk{};
ext_shoot_data_t ShootData{};
ext_bullet_remaining_t BulletRemaining{};
xFrameHeader FrameHeader{};
bool Judge_Data_TF = false;
}

void JUDGE_Link::Judge_Start()
{
    judge_uart = XRobotHardware::JUDGE_Uart();
    if(judge_uart == nullptr)
    {
        return;
    }
    judge_wait_callback = LibXR::Callback<LibXR::ErrorCode>::Create(&JUDGE_Link::Judge_Wait, this);
    judge_wait = LibXR::ReadOperation(judge_wait_callback);
    Arm_Judge_Receive();
}

void JUDGE_Link::Arm_Judge_Receive(bool in_isr)
{
    if (judge_uart != nullptr)
    {
        static_cast<void>(judge_uart->Read(LibXR::RawData(nullptr, 0), judge_wait, in_isr));
    }
}

void JUDGE_Link::Judge_Wait(bool in_isr, JUDGE_Link *self, LibXR::ErrorCode status)
{
    (void)status;
    if (self != nullptr)
    {
        self->judge_ready.PostFromCallback(in_isr);
    }
}

void JUDGE_Link::Judge_Parase()
{
    if (judge_uart == nullptr)
    {
        return;
    }

    LibXR::ReadOperation drain_operation;
    std::array<std::uint8_t, JUDGE_RX_CHUNK_LENGTH> rx_chunk{};
    const std::size_t available = judge_uart->read_port_->Size();
    if (available == 0U)
    {
        return;
    }

    const std::size_t count = std::min(available, rx_chunk.size());
    if (judge_uart->Read({rx_chunk.data(), count}, drain_operation) != LibXR::ErrorCode::OK)
    {
        return;
    }

    for (std::size_t index = 0; index < count; ++index)
    {
        ConsumeByte(rx_chunk[index]);
    }
}

void JUDGE_Link::ConsumeByte(const std::uint8_t byte)
{
    if (frame_size_ == frame_buffer_.size())
    {
        RemovePendingBytes(1);
    }

    frame_buffer_[frame_size_++] = byte;
    ProcessPendingFrames();
}

void JUDGE_Link::ProcessPendingFrames()
{
    while (frame_size_ > 0)
    {
        const auto end = frame_buffer_.begin() + frame_size_;
        const auto sof = std::find(frame_buffer_.begin(), end, JUDGE_FRAME_HEADER);
        if (sof == end)
        {
            frame_size_ = 0;
            return;
        }

        if (sof != frame_buffer_.begin())
        {
            RemovePendingBytes(static_cast<std::size_t>(sof - frame_buffer_.begin()));
            continue;
        }

        if (frame_size_ < LEN_HEADER)
        {
            return;
        }

        if (Verify_CRC8_Check_Sum(frame_buffer_.data(), LEN_HEADER) == 0U)
        {
            RemovePendingBytes(1);
            continue;
        }

        const std::size_t data_length = static_cast<std::size_t>(frame_buffer_[1]) | (static_cast<std::size_t>(frame_buffer_[2]) << 8U);
        if (data_length > JUDGE_MAX_DATA_LENGTH)
        {
            RemovePendingBytes(1);
            continue;
        }

        const std::size_t frame_length = LEN_HEADER + LEN_CMDID + data_length + LEN_TAIL;
        if (frame_size_ < frame_length)
        {
            return;
        }

        if (Verify_CRC16_Check_Sum( frame_buffer_.data(), static_cast<std::uint32_t>(frame_length)) == 0U)
        {
            RemovePendingBytes(1);
            continue;
        }

        Judge_Receive(frame_buffer_.data(), frame_length);
        RemovePendingBytes(frame_length);
    }
}

void JUDGE_Link::RemovePendingBytes(const std::size_t count)
{
    if (count >= frame_size_)
    {
        frame_size_ = 0;
        return;
    }

    std::memmove(frame_buffer_.data(), frame_buffer_.data() + count, frame_size_ - count);
    frame_size_ -= count;
}
std::size_t data_length;
std::uint16_t command_id;
void JUDGE_Link::Judge_Receive(const std::uint8_t* frame, const std::size_t frame_length)
{
    if (frame == nullptr || frame_length < LEN_HEADER + LEN_CMDID + LEN_TAIL)
    {
        return;
    }

    data_length = static_cast<std::size_t>(frame[1]) | (static_cast<std::size_t>(frame[2]) << 8U);
    command_id =static_cast<std::uint16_t>(frame[5]) | (static_cast<std::uint16_t>(frame[6]) << 8U);
    const std::uint8_t* const data = frame + DATA;

    Judge_Data_TF = true;
    switch (command_id)
    {
        case ID_game_state:
            if (data_length == sizeof(GameState))
            {
                std::memcpy(&GameState, data, sizeof(GameState));
            }
            break;
        case ID_power_heat_data:
            if (data_length == sizeof(PowerHeatData))
            {
                std::memcpy(&PowerHeatData, data, sizeof(PowerHeatData));
            }
            break;
        case ID_game_robot_pos:
            if (data_length == sizeof(GameRobotPos))
            {
                std::memcpy(&GameRobotPos, data, sizeof(GameRobotPos));
            }
            break;
        case ID_shoot_data:
            if (data_length == sizeof(ShootData))
            {
                std::memcpy(&ShootData, data, sizeof(ShootData));
                shooter.shooterNum++;
            }
            break;
        case ID_game_robot_state:
            if(data_length == sizeof(GameRobotState))
            {
                std::memcpy(&GameRobotState, data, sizeof(GameRobotState));
            }   
            
        break;

        default:
            break;
    }

}

extern "C"
{
std::uint8_t JUDGE_GetSelfID()
{
    return GameRobotState.robot_id;
}
std::uint16_t JUDGE_GetClientID()
{
    return static_cast<std::uint16_t>(0x0100U + GameRobotState.robot_id);
}

std::uint16_t JUDGE_GetChassisPowerLimit()
{
    return GameRobotState.chassis_power_limit;
}

bool JUDGE_GetShooterOutputState()
{
    return GameRobotState.power_management_shooter_output != 0;
}

bool JUDGE_GetGimbalOutputState()
{
    return GameRobotState.power_management_gimbal_output != 0;
}

std::uint16_t JUDGE_GetHeatLimit()
{
    return GameRobotState.shooter_barrel_heat_limit;
}

std::uint16_t JUDGE_GetShootSpeedLimit()
{
    return 16;
}

std::uint16_t JUDGE_GetRemain_42_Num()
{
    return BulletRemaining.projectile_allowance_42mm;
}

std::uint16_t JUDGE_GetHP()
{
    return GameRobotState.current_HP;
}

void OS_JudgeCallback(void *argument)
{
    (void)argument;
    while (App_IsInitialized() == 0U)
    {
        LibXR::Thread::Sleep(1);
    }
    judge.Judge_Start();
    for (;;)
    {
        if(judge.judge_ready.Wait() == LibXR::ErrorCode::OK)
        {
            judge.Judge_Parase();
            judge.Arm_Judge_Receive();
        }
    }
}

}
