#include "ws2812.hpp"

#include <algorithm>

#include "libxr.hpp"
#include "xr_hardware.hpp"

namespace
{
constexpr uint8_t kLogicalOne = 0xF0U >> 1U;
constexpr uint8_t kLogicalZero = 0xC0U >> 1U;

constexpr uint16_t kDataBytes = 32U;
constexpr uint16_t kResetBytes = 96U;
constexpr uint16_t kDmaBytes = kDataBytes + kResetBytes;

// SPI6 TX 使用 BDMA，发送缓冲区必须位于 BDMA 可访问的 RAM_D3。
__attribute__((used, section(".RAM_D3"), aligned(32)))
uint8_t red_dma_buffer[kDmaBytes] = {};

__attribute__((used, section(".RAM_D3"), aligned(32)))
uint8_t green_dma_buffer[kDmaBytes] = {};

__attribute__((used, section(".RAM_D3"), aligned(32)))
uint8_t blue_dma_buffer[kDmaBytes] = {};

LibXR::Mutex& LedSpiMutex()
{
    static LibXR::Mutex mutex;
    return mutex;
}

LibXR::Semaphore& LedSpiCompletion()
{
    static LibXR::Semaphore completed;
    return completed;
}
}

WS2812Led red_led(red_dma_buffer, kDmaBytes);
WS2812Led green_led(green_dma_buffer, kDmaBytes);
WS2812Led blue_led(blue_dma_buffer, kDmaBytes);

WS2812Led::WS2812Led(uint8_t* dma_buffer, const uint16_t dma_size) : m_dma_buffer(dma_buffer), m_dma_size(dma_size), m_red(0), m_green(0), m_blue(0)
{
}

void WS2812Led::Load()
{
    // WS2812 数据顺序为 GRB，每个 LED 数据位编码为一个 SPI 字节。
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
        m_dma_buffer[bit] =
            ((m_green << bit) & 0x80) != 0 ? kLogicalOne : kLogicalZero;
        m_dma_buffer[8U + bit] =
            ((m_red << bit) & 0x80) != 0 ? kLogicalOne : kLogicalZero;
        m_dma_buffer[16U + bit] =
            ((m_blue << bit) & 0x80) != 0 ? kLogicalOne : kLogicalZero;
    }

    // 数据结束后保持低电平，形成 WS2812 的复位/锁存时间。
    std::fill(m_dma_buffer + 24U, m_dma_buffer + kDmaBytes, 0U);

    LibXR::STM32SPI* spi = XRobotHardware::WS2812Spi();
    if (spi == nullptr)
    {
        return;
    }

    // BLOCK 操作在 LibXR 的 SPI DMA 完成回调后返回，保证缓冲区可安全复用。
    LibXR::WriteOperation operation(LedSpiCompletion());
    const LibXR::ErrorCode status =
        spi->Write(LibXR::ConstRawData(m_dma_buffer, m_dma_size), operation);
    if (status != LibXR::ErrorCode::OK)
    {
        return;
    }
}

void WS2812Led::SetColor(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    // LED tasks may run before app_main() binds the LibXR hardware.
    if (XRobotHardware::WS2812Spi() == nullptr)
    {
        return;
    }

    // 三个颜色对象共用 SPI6 的 LibXR TX 缓冲区，串行化整个写入和完成等待。
    LibXR::Mutex::LockGuard lock(LedSpiMutex());
    m_red = red;
    m_green = green;
    m_blue = blue;
    Load();
}

void WS2812Led::SetRed(const uint8_t brightness)
{
    SetColor(brightness, 0, 0);
}

void WS2812Led::SetGreen(const uint8_t brightness)
{
    SetColor(0, brightness, 0);
}

void WS2812Led::SetBlue(const uint8_t brightness)
{
    SetColor(0, 0, brightness);
}

void WS2812Led::Off()
{
    SetColor(0, 0, 0);
}
