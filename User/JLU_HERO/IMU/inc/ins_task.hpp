#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "semaphore.hpp"
#include <stdint.h>

#define X_axis 0
#define Y_axis 1
#define Z_axis 2
typedef struct
{
    float q[4];

    float Gyro[3];
    float Accel[3];

    float MotionAccel_b[3];
    float MotionAccel_n[3];

    float AccelLPF;

    float xn[3];
    float yn[3];
    float zn[3];

    float Roll;
    float Pitch;
    float Yaw;

} INS_t;

#ifdef __cplusplus
extern "C"
{
#endif

extern INS_t INS;

void INS_Init(void);
void INS_Task(void);


void OS_IMUCallback(void const *argument);

float User_Imu_GetYaw(void);
float User_Imu_GetPitch(void);
float User_Imu_GetRoll(void);

float User_Imu_Get_ROLL_Gyro(void);
float User_Imu_Get_PITCH_Gyro(void);
float User_Imu_Get_YAW_Gyro(void);

#ifdef __cplusplus
}
#endif

#endif
