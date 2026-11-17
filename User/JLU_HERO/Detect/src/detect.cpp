#include "detect.hpp"
#include "RC.hpp"
#include "libxr.hpp"
#include "Beep.hpp"
extern "C"
{
#include "main.h"
#include "cmsis_os.h"
}

DetectDevice detectList[DETECT_DEVICE_NUM]{};
std::atomic_bool g_app_initialized{false};
extern "C" uint8_t App_IsInitialized(void)
{
  return g_app_initialized.load(std::memory_order_acquire) ? 1 : 0;
}
namespace
{

void Detect_DefaultLostHandler(const std::uint8_t deviceID)
{
    (void)deviceID;
}

void Detect_DefaultRecoverHandler(const std::uint8_t deviceID)
{
    (void)deviceID;
}

bool IsValidDeviceId(const std::uint8_t deviceID)
{
    return deviceID < DETECT_DEVICE_NUM;
}

} 

extern "C"
{

void Detect_Update(const std::uint8_t deviceID)
{
    if (!IsValidDeviceId(deviceID))
    {
        return;
    }

    detectList[deviceID].lastRecieveTime = HAL_GetTick();
}

void Detect_InitDevice(const std::uint8_t deviceID, const std::uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void))
{
    if (!IsValidDeviceId(deviceID))
    {
        return;
    }

    detectList[deviceID].maxInterval = maxInterval;

    detectList[deviceID].lastRecieveTime = HAL_GetTick();

    detectList[deviceID].isLost = 0U;
    detectList[deviceID].lastLostState = 0U;
    detectList[deviceID].lostFunc = lostFunc;
    detectList[deviceID].recoverFunc = recoverFunc;
}

void Detect_InitAll(void)
{
    Detect_InitDevice(DeviceID_YawMotor, 50, nullptr, nullptr);
    Detect_InitDevice(DeviceID_PitchMotor, 50, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor1, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor2, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor3, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor4, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor5, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_FricMotor6, 20, nullptr, nullptr);
    Detect_InitDevice(DeviceID_TrigMotor, 50, nullptr, nullptr);
    Detect_InitDevice(DeviceID_RC, 100, RC_Init, nullptr);
    Detect_InitDevice(DeviceID_IMU, 500, nullptr, nullptr);
}

void Task_Detect_Callback(void)
{
    const std::uint32_t presentTime = HAL_GetTick();

    for (std::uint8_t id = 0U; id < DETECT_DEVICE_NUM; ++id)
    {
        DetectDevice &device = detectList[id];

        const std::uint32_t elapsedTime = presentTime - device.lastRecieveTime;

        if (elapsedTime > device.maxInterval && device.isLost == 0)
        {
            device.isLost = 1U;

            if (device.lostFunc == nullptr)
            {
                Detect_DefaultLostHandler(id);
            }
            else
            {
                device.lostFunc();
            }
        }
        else if (elapsedTime <= device.maxInterval && device.isLost == 1)
        {
            if (device.recoverFunc == nullptr)
            {
                Detect_DefaultRecoverHandler(id);
            }
            else
            {
                device.recoverFunc();
            }

            device.isLost = 0;
        }
    }
}

std::uint8_t Detect_IsDeviceLost(const std::uint8_t deviceID)
{
    if (!IsValidDeviceId(deviceID))
    {
        return 1U;
    }

    return detectList[deviceID].isLost;
}

void OS_DetectCallback(void const *argument)
{
    (void)argument;

    while (App_IsInitialized() == 0)
    {
    LibXR::Thread::Sleep(1);
    }

    for (;;)
    {
        Task_Detect_Callback();
        LibXR::Thread::Sleep(5);
    }
}
}
