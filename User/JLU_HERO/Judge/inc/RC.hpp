#pragma once
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "operation.hpp"
#include "uart.hpp"
#include <stdint.h>


extern "C"
{

#include "usart.h"
#include "cmsis_os.h"
#include "main.h"

}

inline constexpr uint8_t rx_length = 50;
inline constexpr uint8_t frame_length = 25;

typedef struct
{
   int16_t ch[16];

    /* 摇杆通道 */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    /* 开关通道 */
    uint8_t SWA;
    uint8_t SWB;
    uint8_t SWC;
    uint8_t SWD;

    /* 旋钮通道 */
    int16_t VRA;
    int16_t VRB;

    /* 接收状态 */
    uint8_t lost;
    uint8_t failsafe;
} RC_TypeDef;

typedef enum
{
    SWITCH_UP = 1,
    SWITCH_MID = 3,
    SWITCH_DOWN = 2
} SwitchState;
class RC_Link
{
    public:
    RC_Link(uint8_t *rx_buffer);
    void Start();
    void Callback(void const *argument);
    void RC_ParseUsart(uint8_t *buff,RC_TypeDef *rc);
    void Arm_RC_Receive(bool in_isr = false);
    float DeadZone(float x, float zone);
    SwitchState SBUS_SwitchState(int16_t ch);
    private:
    static void RcReceive(bool in_isr, RC_Link *self,LibXR::ErrorCode status);
    static void RcWait(bool in_isr,RC_Link *self,LibXR::ErrorCode status);
    private:
    LibXR::UART *r_xr_uart;
    LibXR::Callback<LibXR::ErrorCode> r_read_callback;
    LibXR::Callback<LibXR::ErrorCode> r_wait_callback;
    LibXR::ReadOperation r_read_operation;
    LibXR::ReadOperation r_wait_operation;
};

typedef struct
{
    uint16_t error_cont;
    uint16_t error_dt7;
    uint32_t rc_event;
}RC_status;



extern "C"
{
extern RC_TypeDef rcInfo;
extern uint8_t usart5RxBuf[25];
void RC_Init(void);
void RC_Rx(void);  
void RC_ParseUsart(uint8_t *buff, RC_TypeDef *rc);
void OS_RcCallback(void const *argument);
}


