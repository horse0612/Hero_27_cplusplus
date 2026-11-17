#pragma once

#include <cstdint>

// 基于 Wang Hongxi 的 bsp_dwt V1.1.0 改写。

struct DWT_Time_t
{
    std::uint32_t s;
    std::uint16_t ms;
    std::uint16_t us;
};

extern DWT_Time_t SysTime;
extern std::uint64_t CYCCNT64;

// CPU 主频单位：MHz，例如 480 MHz 传入 480。
void DWT_Init(std::uint32_t cpuFrequencyMHz);

// 返回距离上一次采样经过的时间，单位：秒。
// 每个调用者应维护自己的 lastCount。
float DWT_GetDeltaT(std::uint32_t* lastCount);
double DWT_GetDeltaT64(std::uint32_t* lastCount);

// 更新从 DWT_Init 开始累计的时间。
void DWT_SysTimeUpdate();

// 获取累计时间。
float DWT_GetTimeline_s();
float DWT_GetTimeline_ms();
std::uint64_t DWT_GetTimeline_us();

// 忙等待延时，单位：秒。
// 单次延时必须短于 DWT 32 位计数器的回绕周期。
void DWT_Delay(float seconds);