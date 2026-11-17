#pragma once

extern "C"
{
#include "cmsis_os.h"
#include "spi.h"

void OS_ErrorCallback(void *argument);

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t ErrorTaskHandle;
extern osThreadId_t RS485TaskHandle;
extern osThreadId_t IMUTaskHandle;
extern osThreadId_t GimbalTaskHandle;
extern osThreadId_t ShooterTaskHandle;
extern osThreadId_t VisionTaskHandle;
extern osThreadId_t JudgeTaskHandle;
extern osThreadId_t DetectTaskHandle;
extern osThreadId_t BeepTaskHandle;
extern osThreadId_t RcTaskHandle;
extern osThreadId_t MotorTaskHandle;
extern osThreadId_t LEDTaskHandle;
}
