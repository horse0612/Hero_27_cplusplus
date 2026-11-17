#pragma once

#include <atomic>
#include <cstdint>

enum DetectDeviceId : std::uint8_t
{
    DeviceID_YawMotor=0,
    DeviceID_TrigMotor,
    DeviceID_FricMotor1,
    DeviceID_FricMotor2,
    DeviceID_FricMotor3,
    DeviceID_FricMotor4,
    DeviceID_FricMotor5,
    DeviceID_FricMotor6,
    DeviceID_PitchMotor,
    DeviceID_RC,
    DeviceID_IMU,
    DeviceID_Board2Board,
    DETECT_DEVICE_NUM
};

struct DetectDevice
{
    std::uint32_t maxInterval;
    std::uint32_t lastRecieveTime;
    std::uint8_t isLost;
    std::uint8_t lastLostState;

    void (*lostFunc)(void);
    void (*recoverFunc)(void);
};



extern std::atomic_bool g_app_initialized;

extern "C"
{
uint8_t App_IsInitialized(void);
extern DetectDevice detectList[DETECT_DEVICE_NUM];
void Detect_Update(std::uint8_t deviceID);
void Detect_InitDevice(std::uint8_t deviceID, std::uint32_t maxInterval, void (*lostFunc)(void), void (*recoverFunc)(void));
void Detect_InitAll(void);
void Task_Detect_Callback(void);
std::uint8_t Detect_IsDeviceLost(std::uint8_t deviceID);
void OS_DetectCallback(void const *argument);
}
