#include "ins_task.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "libxr.hpp"
#include "BMI088driver.hpp"
#include "BMI088Middleware.hpp"
#include "semaphore.hpp"
#include "vqf.hpp"
#include "detect.hpp"
extern "C"
{
#include "main.h"
#include "app_main.h"
#include "cmsis_os.h"
#include "tim.h"
}

INS_t INS __attribute__((section(".dtcm_bss"))) = {};
static const float RadToDeg = 57.295779513f;
static const float BodyXAxis[3] = {1.0f, 0.0f, 0.0f};
static const float BodyYAxis[3] = {0.0f, 1.0f, 0.0f};
static const float BodyZAxis[3] = {0.0f, 0.0f, 1.0f};

namespace
{
constexpr vqf_real_t kVQFSamplePeriodS = 0.0005f;
VQFParams InitVQFParams()
{
    VQFParams params;
    params.magDistRejectionEnabled = false;
    params.motionBiasEstEnabled = true;
    params.restBiasEstEnabled = true;
    params.restThGyr = 4.0F;
    params.restThAcc = 0.5F;
    params.restMinT = 1.0F;
    return params;
}

VQF vqf __attribute__((section(".dtcm_cpp"), aligned(8))) = VQF(InitVQFParams(), kVQFSamplePeriodS, kVQFSamplePeriodS);

constexpr size_t kImuQueueCapacity = 32;
LibXR::SPSCQueue<LibXR::Topic::Message<IMU_Data_t>> imu_queue(kImuQueueCapacity);
LibXR::Topic::QueuedSubscriber* imu_subscriber = nullptr;
uint64_t last_imu_timestamp = 0U;
}

static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] + (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] + (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

void INS_Init(void)
{
    if (imu_subscriber == nullptr)
    {
        imu_subscriber = new LibXR::Topic::QueuedSubscriber(BMI088_GetSampleTopic(), imu_queue);
    }

    vqf.resetState();
    last_imu_timestamp = 0U;
    INS.q[0] = 1.0f;
    INS.q[1] = 0.0f;
    INS.q[2] = 0.0f;
    INS.q[3] = 0.0f;
    INS.AccelLPF = 0.0085f;
    static_cast<void>(BMI088_StartAsync());
}

void INS_Task(void)
{
    const float gravity[3] = {0.0f, 0.0f, 9.81f};
    LibXR::Topic::Message<IMU_Data_t> message {};
    while (imu_queue.Pop(message) == LibXR::ErrorCode::OK)
    {
        const uint64_t timestamp = static_cast<uint64_t>(message.timestamp);
        float dt = kVQFSamplePeriodS;
        if (last_imu_timestamp != 0 && timestamp > last_imu_timestamp)
        {
            const uint64_t delta_us = timestamp - last_imu_timestamp;
            if (delta_us >= 400U && delta_us <= 5000U)
            {
                dt = static_cast<float>(delta_us) * 1.0e-6F;
            }
        }
        last_imu_timestamp = timestamp;
      
        const IMU_Data_t& sample = message.data;

        INS.Accel[X_axis] = sample.Accel[X_axis];
        INS.Accel[Y_axis] = sample.Accel[Y_axis];
        INS.Accel[Z_axis] = sample.Accel[Z_axis];

        INS.Gyro[X_axis] = sample.Gyro[X_axis];
        INS.Gyro[Y_axis] = sample.Gyro[Y_axis];
        INS.Gyro[Z_axis] = sample.Gyro[Z_axis];

        vqf.update(INS.Gyro, INS.Accel, dt);
        
        vqf.getQuat6D(INS.q);

        BodyFrameToEarthFrame(BodyXAxis, INS.xn, INS.q);

        BodyFrameToEarthFrame(BodyYAxis, INS.yn, INS.q);

        BodyFrameToEarthFrame(BodyZAxis, INS.zn, INS.q);

        float gravityBody[3];

        EarthFrameToBodyFrame(gravity, gravityBody, INS.q);

        for (uint8_t i = 0; i < 3; ++i)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravityBody[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }

        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);
        INS.Roll = std::atan2(INS.q[0] * INS.q[1] + INS.q[2] * INS.q[3], 0.5f - INS.q[1] * INS.q[1] - INS.q[2] * INS.q[2]) * RadToDeg;
        INS.Pitch = std::asin(-2.0f * (INS.q[1] * INS.q[3] - INS.q[0] * INS.q[2])) * RadToDeg;
        INS.Yaw = std::atan2(INS.q[1] * INS.q[2] + INS.q[0] * INS.q[3], 0.5f - INS.q[2] * INS.q[2] - INS.q[3] * INS.q[3]) * RadToDeg;
    }
    Detect_Update(DeviceID_IMU);
}

void OS_IMUCallback(void const *argument)
{
    (void)argument;
    while (App_IsInitialized() == 0)
    {
    LibXR::Thread::Sleep(1);
    }
    for (;;)
    {
        INS_Task();
        LibXR::Thread::Sleep(1);
    }
}

float User_Imu_GetYaw(void)
{
    return INS.Yaw;
}

float User_Imu_GetPitch(void)
{
    return INS.Pitch;
}

float User_Imu_GetRoll(void)
{
    return INS.Roll;
}

float User_Imu_Get_ROLL_Gyro(void)
{
    return INS.Gyro[X_axis] * RadToDeg;
}

float User_Imu_Get_PITCH_Gyro(void)
{
    return INS.Gyro[Y_axis] * RadToDeg;
}

float User_Imu_Get_YAW_Gyro(void)
{
    return INS.Gyro[Z_axis] * RadToDeg;
}
