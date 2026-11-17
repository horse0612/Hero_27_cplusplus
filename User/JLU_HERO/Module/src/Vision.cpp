#include "Vision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include "detect.hpp"
#include "ins_task.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "usb/device/dev_core.hpp"
#include "User_CAN.hpp"




namespace
{
static_assert(sizeof(VisionReceive) == 14);
static_assert(sizeof(Vision_Transmit) == 16);
LibXR::USB::CDCUart* vision_cdc = nullptr;
LibXR::USB::DeviceCore* vision_usb = nullptr;
LibXR::Thread vision_rx_thread;
std::array<std::uint8_t, sizeof(VisionReceive)> vision_frame{};
std::size_t vision_frame_size = 0U;
extern "C"
{
volatile std::uint8_t VisionUsb_tx_Cont= 0;
VisionReceive vision_receive{};
Vision_Transmit vision_transmit{};
Vision_Gimbal vision{};
}
std::uint16_t VisionCrc16(const std::uint8_t* data, const std::size_t length)
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t index = 0; index < length; ++index)
    {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (std::uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U) : static_cast<std::uint16_t>(crc << 1U);
    }
    return crc;
}

void Vision_ConsumeByte(const std::uint8_t byte)
{
    VisionReceive frame{};
    if (vision_frame_size == 0U && byte != kVisionRxHeader) 
    {
        return;
    }
    vision_frame[vision_frame_size++] = byte;
    if (vision_frame_size != vision_frame.size()) 
    {
        return;
    }

    vision_frame_size = 0;
    const auto received_crc = static_cast<std::uint16_t>(vision_frame[12]) | (static_cast<std::uint16_t>(vision_frame[13]) << 8U);
    if (vision_frame[11] != kVisionRxFinish || received_crc != VisionCrc16(vision_frame.data(), vision_frame.size() - 2))
    {
        return;
    }

    std::memcpy(&frame, vision_frame.data(), sizeof(frame));
    if (!std::isfinite(frame.yaw) || !std::isfinite(frame.pitch))
    {
        return;
    }

    vision_receive = frame;
    vision.pitch = vision_receive.pitch;
    vision.yaw = -vision_receive.yaw;
    vision.fire = vision_receive.fire;
    vision.found = frame.looking_in;
}

void Vision_OnRxReady(const bool in_isr, LibXR::Semaphore* ready, const LibXR::ErrorCode result)
{
    if (result == LibXR::ErrorCode::OK)
    {
        ready->PostFromCallback(in_isr);
    }
}

void Vision_SendAttitude(LibXR::WriteOperation& operation)
{
    if (vision_cdc == nullptr)
    {
        return;
    }
    vision_transmit.roll = User_Imu_GetPitch();
    vision_transmit.yaw = -User_Imu_GetYaw();
    vision_transmit.pitch = -User_Imu_GetRoll();
    vision_transmit.checksum = VisionCrc16(reinterpret_cast<const std::uint8_t*>(&vision_transmit), sizeof(vision_transmit) - sizeof(vision_transmit.checksum));
    std::uint8_t tx_buffer[sizeof(Vision_Transmit)];
    std::memcpy(tx_buffer, &vision_transmit, sizeof(tx_buffer));
    static_cast<void>(vision_cdc->Write({tx_buffer, sizeof(tx_buffer)}, operation));
}
}



void Vision_BindUsb(LibXR::USB::CDCUart& cdc, LibXR::USB::DeviceCore& usb)
{
    vision_cdc = &cdc;
    vision_usb = &usb;
}

extern "C" void Vision_Init(void)
{
    vision_receive = {};
    vision_transmit = {};
    vision = {};
    vision_frame_size = 0;
    vision_transmit.header = kVisionTxHeader;
    vision_transmit.finish = kVisionTxFinish;
    vision_rx_thread.Create<void*>(nullptr, +[](void*)
    {
        Vision_Run();
    }, "vision_rx", 1024, LibXR::Thread::Priority::HIGH);
}

extern "C" void Vision_Run(void)
{
    LibXR::Semaphore rx_ready;
    auto rx_callback = LibXR::ReadOperation::Callback::Create(Vision_OnRxReady, &rx_ready);
    LibXR::ReadOperation rx_notification(rx_callback);
    LibXR::ReadOperation drain_operation;
    std::array<std::uint8_t, 64> rx_buffer{};
    const auto arm_read = [&]()
    {
        if (vision_cdc != nullptr) static_cast<void>(vision_cdc->Read({nullptr, 0}, rx_notification));
    };
    arm_read();

    for (;;)
    {
        if (vision_cdc == nullptr)
        {
            LibXR::Thread::Sleep(1);
            continue;
        }

        if (rx_ready.Wait() == LibXR::ErrorCode::OK)
        {
            std::size_t processed = 0;
            while (processed < rx_buffer.size())
            {
                const std::size_t available = vision_cdc->read_port_->Size();
                if (available == 0)
                {
                    break;
                }
                const std::size_t count = std::min(available, rx_buffer.size() - processed);
                if (vision_cdc->Read({rx_buffer.data(), count}, drain_operation) != LibXR::ErrorCode::OK) 
                {
                    break;
                }
                for (std::size_t i = 0; i < count; ++i) 
                {
                    Vision_ConsumeByte(rx_buffer[i]);
                }
                processed += count;
            }
            arm_read();
            LibXR::Thread::Yield();
        }

    }
}

extern "C" void OS_VisionCallback(void const* argument)
{
    (void)argument;
    while (App_IsInitialized() == 0U)
    {
        LibXR::Thread::Sleep(1);
    }
    LibXR::WriteOperation tx_operation;
    for (;;)
    {
        switch (vision_usb->GetCurrentConfiguration())
        {
            case 0:
            {
                LibXR::Thread::Sleep(1);
                break;
            }
            case 1:
            {
             
                Vision_SendAttitude(tx_operation);
                LibXR::Thread::Sleep(1);
                
            }
            default:
            {
                LibXR::Thread::Sleep(1);
                break;
            }
        }

    }
}
