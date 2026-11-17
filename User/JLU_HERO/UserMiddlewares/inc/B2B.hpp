#pragma once

#include <atomic>
#include <stdint.h>

#include "libxr.hpp"
#include "uart.hpp"

extern "C"
{

extern uint8_t usart2TxBuf[64];
extern uint8_t usart2RxBuf[128];
extern uint8_t STOPFLAG;
extern uint32_t FEEDBACK;
void RS485_Init(void);
void Rs485_Trans(void);
void Rs485_Rec(void);
void B2B_Callback(void);
void OS_Board2BoardCallback(void const *argument);

}


class B2B_Link
{
  public:
    B2B_Link(uint8_t *tx_buffer, uint8_t *rx_buffer);
    void Start();
    void Transmit(bool in_isr = false);
    void Update();
    void Callback();
    void Run(void const *argument);
    void ArmReceive(bool in_isr = false);

  private:
    static void ReadComplete(bool in_isr, B2B_Link *self, LibXR::ErrorCode status);
    void EncodeTransmitFrame();
    bool DecodeReceiveFrame();

  private:
    LibXR::UART *m_xr_uart;
    LibXR::Callback<LibXR::ErrorCode> m_read_callback;
    LibXR::ReadOperation m_read_operation;
    LibXR::WriteOperation m_write_operation;
   

};

extern B2B_Link board2board;


