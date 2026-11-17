#include "xr_hardware.hpp"

#include "fdcan.h"

// 由gpt封装 app_main 绑定的 LibXR 驱动，供业务模块通过稳定接口访问。是本工程早期不熟悉xr构架的产物，于是我就留下来接着用了
//XrobotHardware这种外设容器的方式本来是xr5.x.x的产物，在xr6.x.x中已经被放弃使用
namespace
{
struct BoundHardware
{
    LibXR::STM32UART* rc_uart = nullptr;
    LibXR::STM32UART* judge_uart = nullptr;
    LibXR::STM32UART* board_to_board_uart = nullptr;
    LibXR::STM32SPI* bmi088_spi = nullptr;
    LibXR::STM32SPI* ws2812_spi = nullptr;
    LibXR::STM32GPIO* accel_cs = nullptr;
    LibXR::STM32GPIO* gyro_cs = nullptr;
    LibXR::STM32GPIO* gyro_int = nullptr;
    LibXR::STM32CANFD* can1 = nullptr;
    LibXR::STM32CANFD* can2 = nullptr;
    LibXR::STM32CANFD* can3 = nullptr;
};

BoundHardware g_hardware;
}

namespace XRobotHardware
{

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
          LibXR::STM32CANFD& can3)
{
    g_hardware = {
        &rc_uart,
        &judge_uart,
        &board_to_board_uart,
        &bmi088_spi,
        &ws2812_spi,
        &accel_cs,
        &gyro_cs,
        &gyro_int,
        &can1,
        &can2,
        &can3,
    };
}

LibXR::STM32UART* RC_Uart()
{
    return g_hardware.rc_uart;
}

LibXR::STM32UART* JUDGE_Uart()
{
    return g_hardware.judge_uart;
}

LibXR::STM32UART* BoardToBoardUart()
{
    return g_hardware.board_to_board_uart;
}

LibXR::STM32SPI* BMI088Spi()
{
    return g_hardware.bmi088_spi;
}

LibXR::STM32SPI* WS2812Spi()
{
    return g_hardware.ws2812_spi;
}

LibXR::STM32GPIO* AccelCs()
{
    return g_hardware.accel_cs;
}

LibXR::STM32GPIO* GyroCs()
{
    return g_hardware.gyro_cs;
}

LibXR::STM32GPIO* GyroInt()
{
    return g_hardware.gyro_int;
}

LibXR::STM32CANFD* CanForHandle(FDCAN_HandleTypeDef* handle)
{
    if (handle == &hfdcan1)
    {
        return g_hardware.can1;
    }
    if (handle == &hfdcan2)
    {
        return g_hardware.can2;
    }
    if (handle == &hfdcan3)
    {
        return g_hardware.can3;
    }
    return nullptr;
}
}
