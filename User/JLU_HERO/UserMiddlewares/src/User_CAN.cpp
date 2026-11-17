#include "User_CAN.hpp"
#include "Moto_ctr.hpp"
#include "Shooter.hpp"
#include "Gimbal.hpp"
#include "Vision.hpp"
#include "detect.hpp"
#include "Judge.hpp"
#include "RC.hpp"
#include "can_feedback_mailbox.hpp"
#include <cmath>
#include "semaphore.hpp"
#include "xr_hardware.hpp"
#include "libxr.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "CAN_LK.hpp"
#include <cstdint>
#include <cstring>
#include <array>
extern "C"
{
#include "app_main.h"
#include "fdcan.h"
}
CanState can_state{};


namespace
{

struct DmAngleTracker
{
    bool initialized = false;
    float previous = 0.0f;
    float total = 0.0f;
    void Update(DmRxSnapshot& sample)
    {
        if (!initialized)
        {
            previous = sample.pos;
            total = sample.pos;
            initialized = true;
        }
        else
        {
            total += std::remainder( sample.pos - previous, 2.0F * static_cast<float>(LibXR::PI));
            previous = sample.pos;
        }

        sample.last_pos = previous;
        sample.total_angle = total;
    }
};

struct DjiAngleTracker
{
    bool initialized = false;
    std::int32_t previous = 0;
    std::int32_t total = 0;
    void Update(DjiRxSnapshot& sample)
    {
        const auto current = static_cast<std::int32_t>(sample.angle);
        if (!initialized)
        {
            previous = current;
            total = current;
            initialized = true;
        }
        else
        {
            auto difference = current - previous;
            if (difference > 4096) difference -= 8192;
            if (difference < -4096) difference += 8192;
            total -= difference;
            previous = current;
        }
        sample.total_angle = total;
    }
};

struct CanFeedbackContext
{
    HeroCan::FeedbackMailbox<DjiRxSnapshot, 1> pitch_mailbox;
    HeroCan::FeedbackMailbox<DmRxSnapshot, 1> yaw_mailbox;
    HeroCan::FeedbackMailbox<DmRxSnapshot, 1> trigger_mailbox;
    HeroCan::FeedbackMailbox<DjiRxSnapshot, 6> friction_mailbox;
    Consumed sequence{};
    DjiAngleTracker pitch_angle;
    DmAngleTracker yaw_angle;
    DmAngleTracker trigger_angle;
    bool can_registered = false;
    bool trigger_origin_set = false;
};

CanFeedbackContext can_feedback;

DmRxSnapshot DecodeDmFeedback(const std::uint8_t* data)
{
    DmRxSnapshot sample{};
    sample.id = data[0] & 0x0F;
    sample.state = data[0] >> 4;
    const auto position = static_cast<std::uint16_t>( (static_cast<std::uint16_t>(data[1]) << 8) | data[2]);
    const auto velocity = static_cast<std::uint16_t>( (static_cast<std::uint16_t>(data[3]) << 4) | (data[4] >> 4));
    const auto torque = static_cast<std::uint16_t>( (static_cast<std::uint16_t>(data[4] & 0x0F) << 8) | data[5]);
    sample.pos = uint_to_float(static_cast<std::int16_t>(position), P_MIN, P_MAX, 16);
    if (sample.pos < 0.0f)
    {
        sample.pos += 2.0f * static_cast<float>(LibXR::PI);    // 映射到 [0, 2π)，因为uint_to_float中存在偏置
    }
    sample.vel = uint_to_float(velocity, V_MIN, V_MAX, 12);
    sample.tor = uint_to_float(torque, T_MIN, T_MAX, 12);
    sample.Tmos = static_cast<float>(data[6]);
    sample.Tcoil = static_cast<float>(data[7]);
    sample.timestamp_us = static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
    return sample;
}

void LPF(DJImotor *motor,float gama)
{
     motor->speed = gama * motor->speed + (1.0f - gama) * motor->lastSpeed;
     motor->lastSpeed = motor->speed;
}

void CAN1_XRUser_Callback(bool, void*,  const LibXR::FDCAN::FDPack& pack)
{
    if (pack.len < 8)
    {
        return;
    }   
    switch (pack.id)
    {
    case 0x14:
    {
        auto yaw_sample = DecodeDmFeedback(pack.data);
        can_feedback.yaw_angle.Update(yaw_sample);
        yaw_sample.sequence = ++can_feedback.sequence.yaw_sequence;
        static_cast<void>(can_feedback.yaw_mailbox.Publish(0, yaw_sample));
        break;
    }

    case 0x12:
    {
        auto trig_sample = DecodeDmFeedback(pack.data);
        if (!can_feedback.trigger_origin_set && trig_sample.pos > 0.0f)
        {
            can_feedback.trigger_origin_set = true;
        }
        can_feedback.trigger_angle.Update(trig_sample);
        trig_sample.sequence = ++can_feedback.sequence.trigger_sequence;
        static_cast<void>(can_feedback.trigger_mailbox.Publish(0, trig_sample));
        break;
    }
    default:
        break;
    }
}

void CAN2_XRUser_Callback(bool, void*, const LibXR::CAN::ClassicPack& pack)
{
    DjiRxSnapshot sample{};
    if (pack.id < 0x201 || pack.id > 0x207) 
    {
         return;
    }
    switch (pack.id)
    {
        case 0x201:
        case 0x202:
        case 0x203:
        {
            const auto index = static_cast<std::uint8_t>(pack.id - 0x201);
            sample.angle = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[0]) << 8) | pack.data[1]);
            sample.speed = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[2]) << 8) | pack.data[3]);
            sample.torque = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[4]) << 8) | pack.data[5]);
            sample.temperature = static_cast<std::int8_t>(pack.data[6]);
            sample.sequence = ++can_feedback.sequence.friction_sequence[index];
            sample.timestamp_us = static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
            sample.num_id = index;
            static_cast<void>(can_feedback.friction_mailbox.Publish(index, sample));
            break;
        }
        case 0x205:
        case 0x206:
        case 0x207:
        {
            const auto index = static_cast<std::uint8_t>(pack.id - 0x202);
            sample.angle = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[0]) << 8) | pack.data[1]);
            sample.speed = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[2]) << 8) | pack.data[3]);
            sample.torque = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[4]) << 8) | pack.data[5]);
            sample.temperature = static_cast<std::int8_t>(pack.data[6]);
            sample.sequence = ++can_feedback.sequence.friction_sequence[index];
            sample.timestamp_us = static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
            sample.num_id = index;
            static_cast<void>(can_feedback.friction_mailbox.Publish(index, sample));
            break;
        }
            
        default:
            return;
    }
    
}

void CAN3_XRUser_Callback(bool, void*, const LibXR::CAN::ClassicPack& pack)
{
    if(pack.dlc != 8)
    {
        return;
    }
    switch (pack.id)
    {
        case 0x207:
        {
         DjiRxSnapshot sample{};
         sample.angle = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[0]) << 8) | pack.data[1]);
         sample.speed = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[2]) << 8) | pack.data[3]);
         sample.torque = static_cast<std::int16_t>((static_cast<std::uint16_t>(pack.data[4]) << 8) | pack.data[5]);
         sample.temperature = static_cast<std::int8_t>(pack.data[6]);
         can_feedback.pitch_angle.Update(sample);
         sample.sequence = ++can_feedback.sequence.pitch_sequence;
         sample.timestamp_us = static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
         static_cast<void>(can_feedback.pitch_mailbox.Publish(0, sample));
        }
    }
}

}

void CAN_RefreshGimbalFeedback()
{
    static std::uint32_t consumed_pitch = 0;
    static std::uint32_t consumed_yaw = 0;
    DjiRxSnapshot pitch_sample{};
    DmRxSnapshot yaw_sample{};
    if (can_feedback.pitch_mailbox.ReadLatest(0, pitch_sample) && pitch_sample.sequence != consumed_pitch)
    {
        consumed_pitch = pitch_sample.sequence;
        DJIMotor_Update(&gimbal.dji6020_pitch, pitch_sample.angle, pitch_sample.speed, pitch_sample.torque, pitch_sample.temperature);
        gimbal.dji6020_pitch.lastAngle = pitch_sample.angle;
        gimbal.dji6020_pitch.totalAngle = pitch_sample.total_angle;
        gimbal.pitchFeedbackValid = true;
    }
    if(can_feedback.yaw_mailbox.ReadLatest(0, yaw_sample) && yaw_sample.sequence != consumed_yaw)
    {
        consumed_yaw = yaw_sample.sequence;
        gimbal.dm8518_yaw.state = yaw_sample.state;
        gimbal.dm8518_yaw.vel = yaw_sample.vel;
        gimbal.dm8518_yaw.totalangle = yaw_sample.total_angle;
        gimbal.yawFeedbackValid = true;
    }

}

void CAN_RefreshShooterFeedback()
{
    static std::uint32_t consumed_trigger = 0;
    static std::uint32_t consumed_friction[6] = {};
    DmRxSnapshot trigger{};
    DjiRxSnapshot fric_sample{};
    if (can_feedback.trigger_mailbox.ReadLatest(0, trigger) && trigger.sequence != consumed_trigger)
    {
        consumed_trigger = trigger.sequence;
        shooter.dm4310_trig.id = trigger.id;
        shooter.dm4310_trig.state = trigger.state;
        shooter.dm4310_trig.pos = trigger.pos;
        shooter.dm4310_trig.vel = trigger.vel;
        shooter.dm4310_trig.tor = trigger.tor;
        shooter.dm4310_trig.Tmos = trigger.Tmos;
        shooter.dm4310_trig.Tcoil = trigger.Tcoil;
        shooter.dm4310_trig.totalangle = trigger.total_angle;
        if (!start_dm3410)
        {
            shooter.dm4310_trig.total_target_pos = trigger.total_angle;
            start_dm3410 = true;
        }
    }
    for(std::uint8_t i = 0; i < 6; ++i)
    {
        if(can_feedback.friction_mailbox.ReadLatest(i, fric_sample) && fric_sample.sequence != consumed_friction[i])
        {
            consumed_friction[i] = fric_sample.sequence;
            DJIMotor_Update(&shooter.fricMotor[i], fric_sample.angle, fric_sample.speed, fric_sample.torque, fric_sample.temperature);
            LPF(&shooter.fricMotor[i], 0.8f);
        }
    }
    
  
    
}


extern "C"
{
void CAN_Init(void)
{
    if (can_feedback.can_registered)
    {
        return;
    }
    auto* can1 = XRobotHardware::CanForHandle(&hfdcan1);
    auto* can2 = XRobotHardware::CanForHandle(&hfdcan2);
    auto* can3 = XRobotHardware::CanForHandle(&hfdcan3);
    if (can1 == nullptr || can2 == nullptr || can3 == nullptr)
    {
        return;
    }
    can1->Register( LibXR::FDCAN::CallbackFD::Create( &CAN1_XRUser_Callback, static_cast<void*>(nullptr)), LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE, 0x000U, 0x7FFU);
    can2->Register( LibXR::CAN::Callback::Create( &CAN2_XRUser_Callback, static_cast<void*>(nullptr)), LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE, 0x000U, 0x7FFU);
    can3->Register( LibXR::CAN::Callback::Create( &CAN3_XRUser_Callback, static_cast<void*>(nullptr)), LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE, 0x000U, 0x7FFU);
    can_feedback.can_registered = true;
}
}



extern "C" void USER_CAN_Send(FDCAN_HandleTypeDef *handle, std::int16_t standardId, std::uint8_t txData[8])
{
    auto *can = XRobotHardware::CanForHandle(handle);
    uint8_t can_num = 0;
    if (handle == &hfdcan1) 
    { 
        can_num = 1; 
    }
    else if (handle == &hfdcan2) 
    { 
        can_num = 2; 
    }
    else if (handle == &hfdcan3) 
    { 
        can_num = 3; 
    }
   
    switch(can_num)
    {
        case 1:
        {
            if (can != nullptr && txData != nullptr && standardId >= 0 && standardId <= 0x7FF)
            {
            auto *fdcan = static_cast<LibXR::FDCAN*>(can);
            LibXR::FDCAN::FDPack pack{};
            pack.id = static_cast<std::uint32_t>(standardId);
            pack.type = LibXR::CAN::Type::STANDARD;
            pack.len = 8U;
            std::memcpy(pack.data, txData, 8);
            if (fdcan->AddMessage(pack) == LibXR::ErrorCode::OK)
            {
                return;
            }
            break;
            }
        }
        case 2:
        case 3:
        {
            if (can != nullptr && txData != nullptr && standardId >= 0 && standardId <= 0x7FF)
            {
            LibXR::CAN::ClassicPack pack{};
            pack.id = static_cast<std::uint32_t>(standardId);
            pack.type = LibXR::CAN::Type::STANDARD;
            pack.dlc = 8U;
            std::memcpy(pack.data, txData, sizeof(pack.data));
            if (can->AddMessage(pack) == LibXR::ErrorCode::OK)
            {
                return;
            }
            break;
            }
    }
   
    }
    if (handle == &hfdcan1) 
    { 
        ++can_state.can1_send_error; 
    }
    else if (handle == &hfdcan2) 
    { 
        ++can_state.can2_send_error; 
    }
    else if (handle == &hfdcan3) 
    { 
        ++can_state.can3_send_error; 
    }
}



