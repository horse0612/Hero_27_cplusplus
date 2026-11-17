#include "ErrorTask.hpp"
#include "fdcan.h"
#include "ws2812.hpp"
#include "LED.hpp"
#include "RC.hpp"
#include "B2B.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "CAN_LK.hpp"
#include "User_CAN.hpp"
#include "Gimbal.hpp"
#include "ins_task.hpp"
extern "C"
{
#include "main.h"
#include "app_main.h"
#include "cmsis_os.h"
}

extern "C" void OS_ErrorCallback(void *argument)
{
    (void)argument;
    osThreadSuspend(ErrorTaskHandle);
    osThreadSuspend(MotorTaskHandle);
    osThreadSuspend(GimbalTaskHandle);
    osThreadSuspend(ShooterTaskHandle);
    osThreadSuspend(DetectTaskHandle);
    osThreadSuspend(BeepTaskHandle);
    osThreadSuspend(LEDTaskHandle);
    osThreadSuspend(VisionTaskHandle);
    osThreadSuspend(RcTaskHandle);
    for (;;)
    {
    DMstop(&hfdcan1, 0x02);
    USER_CAN_SetMotorCurrent(&hfdcan2,0x200,0,0,0,0);
    HAL_Delay(1);
    USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,0,0,0,0);
    HAL_Delay(1);
    DMerror(&hfdcan1, 0x02);
    Rs485_Trans();
    if (rcInfo.SWD != 2	)
    {
      HAL_NVIC_SystemReset(); // 右拨杆回到中间重启系统
    }
        gimbal.pitch.angle = -User_Imu_GetRoll();
        gimbal.yaw_up.angle = User_Imu_GetYaw();
        CAN_RefreshShooterFeedback();
        CAN_RefreshGimbalFeedback();
        red_led.SetRed(64);
        HAL_Delay(100);
        blue_led.Off();
        HAL_Delay(100);
    }
}
