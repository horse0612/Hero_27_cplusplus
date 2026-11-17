#pragma once

#include <cstdint>

class WS2812Led
{
public:
    WS2812Led(uint8_t* dma_buffer, uint16_t dma_size);

    void SetColor(uint8_t red, uint8_t green, uint8_t blue);
    void SetRed(uint8_t brightness = 64);
    void SetGreen(uint8_t brightness = 64);
    void SetBlue(uint8_t brightness = 64);
    void Off();

private:
    void Load();

    uint8_t* m_dma_buffer;
    uint16_t m_dma_size;
    uint8_t m_red;
    uint8_t m_green;
    uint8_t m_blue;
};

// 三个独立的颜色对象，后续任务可以分别控制红、绿、蓝灯。
extern WS2812Led red_led;
extern WS2812Led green_led;
extern WS2812Led blue_led;
