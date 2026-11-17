#include "Moto_ctr.hpp"
#include "Gimbal.hpp"
#include "Shooter.hpp"
#include "User_CAN.hpp"
#include "Judge.hpp"
#include "libxr.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "CAN_LK.hpp"
#include "app_main.h"
#include "detect.hpp"
extern "C"
{
#include "cmsis_os.h"
}

extern "C"
{

void Task_CANMotors_Callback(void)
{

    USER_CAN_SetMotorCurrent(&hfdcan2,0x200,shooter.fricMotor[0].singlePID.output,shooter.fricMotor[1].singlePID.output,shooter.fricMotor[2].singlePID.output,0);

    switch (gimbal.dm8518_yaw.state)
    {
        case 0:
            DMstrat(&hfdcan1, 0x04);
        break;
        case 1:
            MIT_CtrlMotor(&hfdcan1, 0x04, 0.0f, gimbal.yaw_up.imuPID.deOuter.output, 0.0f, 2.0f, 0);
        break;
        default:
            DMerror(&hfdcan1, 0x04);
        break;
    }

    USER_CAN_SetMotorCurrent(&hfdcan3,0x1FF,0,0,gimbal.pitch.imuPID.output,0);

    switch (shooter.dm4310_trig.state)
    {
        case 0:
            DMstrat(&hfdcan1, 0x02);
        break;
        case 1:
            MIT_CtrlMotor(&hfdcan1, 0x02, 0.0f, shooter.dm4310_trig.dePID.output, 0.0f, 2.0f, DM4310_Trig_StaticTorque);
        break;
        default:
            DMerror(&hfdcan1, 0x02);
        break;
    }

    USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,shooter.fricMotor[3].singlePID.output,shooter.fricMotor[4].singlePID.output,shooter.fricMotor[5].singlePID.output,0);
    LibXR::Thread::Sleep(1);
}

void OS_MotorCallback(void const *argument)
{
    (void)argument;

    while (App_IsInitialized() == 0)
    {
        LibXR::Thread::Sleep(1);
    }
    for (;;)
    {
        Task_CANMotors_Callback();
    }
}
}
