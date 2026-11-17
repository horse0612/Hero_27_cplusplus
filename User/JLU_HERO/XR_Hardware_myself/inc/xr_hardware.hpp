#pragma once

#include "libxr.hpp"
#include "stm32_canfd.hpp"
#include "stm32_gpio.hpp"
#include "stm32_spi.hpp"
#include "stm32_uart.hpp"

namespace XRobotHardware
{

/** Bind the concrete LibXR drivers created by app_main(). */
void Bind(LibXR::STM32UART& rc_uart,
          LibXR::STM32UART& judge_uart,
          LibXR::STM32UART& board_to_board_uart,
          LibXR::STM32SPI& bmi088_spi,
          LibXR::STM32SPI& ws2812_spi,
          LibXR::STM32GPIO& accel_cs,
          LibXR::STM32GPIO& gyro_cs,
          LibXR::STM32GPIO& gyro_int,
          LibXR::STM32CANFD& can1,
          LibXR::STM32CANFD& can2,
          LibXR::STM32CANFD& can3);

LibXR::STM32UART* RC_Uart();
LibXR::STM32UART* JUDGE_Uart();
LibXR::STM32UART* BoardToBoardUart();
LibXR::STM32SPI* BMI088Spi();
LibXR::STM32SPI* WS2812Spi();
LibXR::STM32GPIO* AccelCs();
LibXR::STM32GPIO* GyroCs();
LibXR::STM32GPIO* GyroInt();

LibXR::STM32CANFD* CanForHandle(FDCAN_HandleTypeDef* handle);

}  
