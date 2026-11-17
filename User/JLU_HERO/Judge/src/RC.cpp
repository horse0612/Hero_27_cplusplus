#include "RC.hpp"
#include "detect.hpp"
#include "B2B.hpp"
#include "ErrorTask.hpp"
#include "libxr.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "operation.hpp"
#include "xr_hardware.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
alignas(32) std::uint8_t usart5RxBuf[25]{};
extern "C"
{
RC_TypeDef rcInfo = {{0},0};
RC_status rcStatus = {0};
}
volatile size_t read_len = 0;
RC_Link rc(usart5RxBuf);

RC_Link::RC_Link(uint8_t *rx_buffer):r_xr_uart(nullptr),r_read_callback(),r_wait_callback(),r_read_operation(),r_wait_operation() {}

void RC_Link::RcWait(bool in_isr,RC_Link *self,LibXR::ErrorCode status)
{
    if(status != LibXR::ErrorCode::OK)
    {
        rcStatus.error_cont++;
        return;
    }

    const size_t queue_available = self->r_xr_uart->read_port_->Size();
    read_len = queue_available > rx_length ? rx_length  : queue_available;
    (void)self->r_xr_uart->Read(LibXR::RawData(usart5RxBuf,read_len), self->r_read_operation,in_isr);
}

void RC_Link::RcReceive(bool in_isr,RC_Link *self,LibXR::ErrorCode status)
{
   (void)in_isr;
   if(status != LibXR::ErrorCode::OK)
   {
       rcStatus.error_dt7++;
       return;
   }
   rcStatus.rc_event++;
   const size_t size = read_len;
   if(size >= frame_length)
   {
       const size_t offset = size -(size % frame_length) - frame_length;
       self->RC_ParseUsart(&usart5RxBuf[offset], &rcInfo);
   }
   if(self->r_xr_uart != nullptr)
   {
       self->Arm_RC_Receive(in_isr);
   }
}

void RC_Link::Arm_RC_Receive(bool in_isr)
{
    (void)r_xr_uart->Read(LibXR::RawData(nullptr,0), r_wait_operation,in_isr);
}

void RC_Link::Start()
{
    r_xr_uart = XRobotHardware::RC_Uart();
    if (r_xr_uart == nullptr)
    {
        return;
    }
    r_read_callback = LibXR::Callback<LibXR::ErrorCode>::Create(&RC_Link::RcReceive, this);
    r_wait_callback = LibXR::Callback<LibXR::ErrorCode>::Create(&RC_Link::RcWait,this);
    r_read_operation = LibXR::ReadOperation(r_read_callback);
    r_wait_operation = LibXR::ReadOperation(r_wait_callback);
    Arm_RC_Receive();
}

float RC_Link::DeadZone(float x, float zone)
{
     if (x > -zone && x < zone)
        return 0.0f;
    return x;
}

SwitchState RC_Link::SBUS_SwitchState(int16_t ch)
{
     if (ch > 1500)       return SWITCH_DOWN;
    else if (ch < 500) return SWITCH_UP;
    else                return SWITCH_MID;
}

void RC_Link::RC_ParseUsart(uint8_t *buff, RC_TypeDef *rc)
{
    if (buff == NULL || rc == NULL)
    {
        return;
    }
    if (buff[0] != 0x0F || buff[24] != 0x00)
    {
        return;
    }
    rc->ch[0]  = ((buff[1]      | buff[2]  << 8) & 0x07FF);
    rc->ch[1]  = ((buff[2] >> 3 | buff[3]  << 5) & 0x07FF);
    rc->ch[2]  = ((buff[3] >> 6 | buff[4]  << 2 | buff[5] << 10) & 0x07FF);
    rc->ch[3]  = ((buff[5] >> 1 | buff[6]  << 7) & 0x07FF);
    rc->ch[4]  = ((buff[6] >> 4 | buff[7]  << 4) & 0x07FF);
    rc->ch[5]  = ((buff[7] >> 7 | buff[8]  << 1 | buff[9] << 9) & 0x07FF);
    rc->ch[6]  = ((buff[9] >> 2 | buff[10] << 6) & 0x07FF);
    rc->ch[7]  = ((buff[10] >> 5 | buff[11] << 3) & 0x07FF);

    rc->ch[8]  = ((buff[12]      | buff[13] << 8) & 0x07FF);
    rc->ch[9]  = ((buff[13] >> 3 | buff[14] << 5) & 0x07FF);
    rc->ch[10] = ((buff[14] >> 6 | buff[15] << 2 | buff[16] << 10) & 0x07FF);
    rc->ch[11] = ((buff[16] >> 1 | buff[17] << 7) & 0x07FF);
    rc->ch[12] = ((buff[17] >> 4 | buff[18] << 4) & 0x07FF);
    rc->ch[13] = ((buff[18] >> 7 | buff[19] << 1 | buff[20] << 9) & 0x07FF);
    rc->ch[14] = ((buff[20] >> 2 | buff[21] << 6) & 0x07FF);
    rc->ch[15] = ((buff[21] >> 5 | buff[22] << 3) & 0x07FF);

    rc->ch1 = -(rc->ch[0] - 1024); //右横        
    rc->ch2 = rc->ch[1] - 1024; //右竖
    rc->ch3 = rc->ch[2] - 1024; //左竖
    rc->ch4 = -(rc->ch[3] - 1024); //左横
    rc->ch1 = DeadZone(rc->ch1,10);
    rc->ch2 = DeadZone(rc->ch2,10);
    rc->ch3 = DeadZone(rc->ch3,10);
    rc->ch4 = DeadZone(rc->ch4,10);
    rc->SWA = SBUS_SwitchState(rc->ch[4]);
    rc->SWB = SBUS_SwitchState(rc->ch[5]);
    rc->SWC = SBUS_SwitchState(rc->ch[6]);
    rc->SWD = SBUS_SwitchState(rc->ch[7]);
    rc->VRA = rc->ch[8] - 240;
    rc->VRB = rc->ch[9] - 240;
    
    rc->lost = (buff[23] & (1 << 2)) ? 1 : 0;
    rc->failsafe = (buff[23] & (1 << 3)) ? 1 : 0;
    Detect_Update(DeviceID_RC);    
}


void RC_Link::Callback(void const *argument)
{
    (void)argument;
    while (App_IsInitialized() == 0)
    {
        LibXR::Thread::Sleep(1);
    }
    for(;;)
    {
        if (rcInfo.SWD == 2)
        {
            osThreadResume(ErrorTaskHandle);
        }
        LibXR::Thread::Sleep(15);
    }
}

extern "C" void RC_Rx(void)
{
    rc.Arm_RC_Receive();
}

extern "C" void RC_Init(void)
{
    rc.Start();
}

extern "C" void OS_RcCallback(void const *argument)
{
   rc.Callback(argument);
}




