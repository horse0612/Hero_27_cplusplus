#include "B2B.hpp"
#include "detect.hpp"
#include "xr_hardware.hpp"
#include "ErrorTask.hpp"
#include <cstring>

extern "C"
{
#include "main.h"
#include "cmsis_os.h"
#include "stdint.h"
}


extern "C"
{
uint8_t usart2TxBuf[64];
uint8_t usart2RxBuf[128];
uint8_t STARTFLAG = 0;
uint32_t FEEDBACK = 0;

uint8_t STOPFLAG = 0;
}

B2B_Link board2board(usart2TxBuf, usart2RxBuf);

namespace
{
constexpr uint32_t kB2BRequestPeriodMs = 1U;
}

B2B_Link::B2B_Link(uint8_t *tx_buffer, uint8_t *rx_buffer) : m_xr_uart(nullptr), m_read_callback(), m_read_operation(), m_write_operation() {}

void B2B_Link::ReadComplete(bool in_isr, B2B_Link *self, LibXR::ErrorCode status)
{
    if (self == nullptr || status != LibXR::ErrorCode::OK)
    {
        if (self != nullptr)
        {
            self->ArmReceive(in_isr);
        }
        return;
    }

    (void)self->DecodeReceiveFrame();
    self->ArmReceive(in_isr);
}

void B2B_Link::ArmReceive(bool in_isr)
{
    (void)m_xr_uart->Read(LibXR::RawData(usart2RxBuf, 64), m_read_operation, in_isr);
}

void B2B_Link::Start()
{
    m_xr_uart = XRobotHardware::BoardToBoardUart();
    if (m_xr_uart == nullptr)
    {
        return;
    }
    m_read_callback = LibXR::Callback<LibXR::ErrorCode>::Create( &B2B_Link::ReadComplete, this);
    m_read_operation = LibXR::ReadOperation(m_read_callback);
    ArmReceive();
}

void B2B_Link::EncodeTransmitFrame()
{
    std::memset(usart2TxBuf, 0, 64U);

    usart2TxBuf[0] = 0xAA;
    // 在这里填写上板发送给下板的数据。
    usart2TxBuf[1] = STARTFLAG;
    usart2TxBuf[2] = STOPFLAG ;
    usart2TxBuf[63] = 0xFE;
}
void B2B_Link::Transmit(bool in_isr)
{

    STARTFLAG = 1;
    EncodeTransmitFrame();
    (void)m_xr_uart->Write( LibXR::ConstRawData(usart2TxBuf, 64), m_write_operation, in_isr);
}

bool B2B_Link::DecodeReceiveFrame()
{
    if (usart2RxBuf[0] != 0xAB || usart2RxBuf[63] != 0xFD)
    {
        return false;
    }

    // 在这里解析下板回复的数据。
    FEEDBACK = static_cast<uint32_t>(usart2RxBuf[1]) |
               (static_cast<uint32_t>(usart2RxBuf[2]) << 8U) |
               (static_cast<uint32_t>(usart2RxBuf[3]) << 16U) |
               (static_cast<uint32_t>(usart2RxBuf[4]) << 24U);
    Detect_Update(DeviceID_Board2Board);
    return true;
}

void B2B_Link::Update()
{
    (void)DecodeReceiveFrame();
}

void B2B_Link::Callback()
{
    Transmit();
}

void B2B_Link::Run(void const *argument)
{
    (void)argument;
    
    while (App_IsInitialized() == 0)
    {
        LibXR::Thread::Sleep(1);
    }

    if(STOPFLAG == 1)
    {
        osThreadResume(ErrorTaskHandle);
    }

    for (;;)
    {
        // 上板按固定周期轮询下板；回复由 ReadComplete() 接收和解析。
        Callback();
        LibXR::Thread::Sleep(kB2BRequestPeriodMs);
    }
}



extern "C" void RS485_Init(void)
{
    board2board.Start();
}

extern "C" void Rs485_Trans(void)
{
    board2board.Transmit();
}

extern "C" void Rs485_Rec(void)
{
    board2board.ArmReceive();
}

extern "C" void B2B_Callback(void)
{
    board2board.Callback();
}

extern "C" void OS_Board2BoardCallback(void const *argument)
{
    board2board.Run(argument);
}
