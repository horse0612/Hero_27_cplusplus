#include "bsp_dwt.hpp"

#include <cmath>

extern "C"
{
#include "main.h"
}

// 基于 Wang Hongxi 的 bsp_dwt V1.1.0 改写。

DWT_Time_t SysTime{};
std::uint64_t CYCCNT64 = 0U;

namespace
{

std::uint32_t cpuFrequencyHz = 0U;
std::uint32_t cyclesPerMillisecond = 0U;
std::uint32_t cyclesPerMicrosecond = 0U;

std::uint32_t lastCycleCount = 0U;
std::uint64_t cycleOverflowCount = 0U;

constexpr std::uint64_t CycleCounterPeriod =
    std::uint64_t{1} << 32U;

// 使用同一次寄存器采样判断回绕、计算累计周期数。
std::uint64_t UpdateCycleCount()
{
    const std::uint32_t currentCount = DWT->CYCCNT;

    if (currentCount < lastCycleCount)
    {
        ++cycleOverflowCount;
    }

    lastCycleCount = currentCount;

    CYCCNT64 =
        cycleOverflowCount * CycleCounterPeriod +
        static_cast<std::uint64_t>(currentCount);

    return CYCCNT64;
}

}  // namespace

void DWT_Init(const std::uint32_t cpuFrequencyMHz)
{

    cpuFrequencyHz = cpuFrequencyMHz * 1'000'000U;
    cyclesPerMillisecond = cpuFrequencyHz / 1'000U;
    cyclesPerMicrosecond = cpuFrequencyHz / 1'000'000U;

    // 使能 Cortex-M 跟踪功能。
    CoreDebug->DEMCR =
        CoreDebug->DEMCR | CoreDebug_DEMCR_TRCENA_Msk;

    // 暂停计数，然后清零。
    DWT->CTRL =
        DWT->CTRL & ~DWT_CTRL_CYCCNTENA_Msk;

    DWT->CYCCNT = 0U;

    lastCycleCount = 0U;
    cycleOverflowCount = 0U;
    CYCCNT64 = 0U;
    SysTime = {};

    // 启动周期计数器。
    DWT->CTRL =
        DWT->CTRL | DWT_CTRL_CYCCNTENA_Msk;

    __DSB();
    __ISB();
}

float DWT_GetDeltaT(std::uint32_t* lastCount)
{
    if (lastCount == nullptr || cpuFrequencyHz == 0U)
    {
        return 0.0f;
    }

    const std::uint32_t currentCount = DWT->CYCCNT;

    // 无符号减法可以处理一次计数器回绕。
    const std::uint32_t elapsedCycles =
        currentCount - *lastCount;

    *lastCount = currentCount;

    UpdateCycleCount();

    return static_cast<float>(elapsedCycles) /
           static_cast<float>(cpuFrequencyHz);
}

double DWT_GetDeltaT64(std::uint32_t* lastCount)
{
    if (lastCount == nullptr || cpuFrequencyHz == 0U)
    {
        return 0.0;
    }

    const std::uint32_t currentCount = DWT->CYCCNT;

    const std::uint32_t elapsedCycles =
        currentCount - *lastCount;

    *lastCount = currentCount;

    UpdateCycleCount();

    return static_cast<double>(elapsedCycles) /
           static_cast<double>(cpuFrequencyHz);
}

void DWT_SysTimeUpdate()
{
    if (cpuFrequencyHz == 0U)
    {
        return;
    }

    const std::uint64_t totalCycles = UpdateCycleCount();

    const std::uint64_t seconds =
        totalCycles / cpuFrequencyHz;

    const std::uint64_t remainingCycles =
        totalCycles % cpuFrequencyHz;

    SysTime.s = static_cast<std::uint32_t>(seconds);

    SysTime.ms = static_cast<std::uint16_t>(
        remainingCycles / cyclesPerMillisecond);

    SysTime.us = static_cast<std::uint16_t>(
        (remainingCycles % cyclesPerMillisecond) /
        cyclesPerMicrosecond);
}

float DWT_GetTimeline_s()
{
    DWT_SysTimeUpdate();

    return static_cast<float>(SysTime.s) +
           static_cast<float>(SysTime.ms) * 0.001f +
           static_cast<float>(SysTime.us) * 0.000001f;
}

float DWT_GetTimeline_ms()
{
    DWT_SysTimeUpdate();

    return static_cast<float>(SysTime.s) * 1'000.0f +
           static_cast<float>(SysTime.ms) +
           static_cast<float>(SysTime.us) * 0.001f;
}

std::uint64_t DWT_GetTimeline_us()
{
    DWT_SysTimeUpdate();

    return static_cast<std::uint64_t>(SysTime.s) *
               1'000'000ULL +
           static_cast<std::uint64_t>(SysTime.ms) *
               1'000ULL +
           static_cast<std::uint64_t>(SysTime.us);
}

void DWT_Delay(const float seconds)
{
    if (cpuFrequencyHz == 0U ||
        !std::isfinite(seconds) ||
        seconds <= 0.0f)
    {
        return;
    }

    const double requestedCycles =
        static_cast<double>(seconds) *
        static_cast<double>(cpuFrequencyHz);

    // 本接口使用 32 位差值，不能延时一个完整回绕周期。
    if (requestedCycles >=
        static_cast<double>(CycleCounterPeriod))
    {
        return;
    }

    const std::uint32_t waitCycles =
        static_cast<std::uint32_t>(
            std::ceil(requestedCycles));

    const std::uint32_t startCount = DWT->CYCCNT;

    while (static_cast<std::uint32_t>(
               DWT->CYCCNT - startCount) < waitCycles)
    {
        // 更新累计时间，避免延时期间漏掉一次回绕。
        UpdateCycleCount();
    }

    UpdateCycleCount();
}