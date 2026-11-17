#include "Shooter.hpp"

#include <cmath>

#include "Gimbal.hpp"
#include "RC.hpp"
#include "Vision.hpp"
#include "User_CAN.hpp"
#include "detect.hpp"
#include "libxr.hpp"
#include "semaphore.hpp"
LibXR::Semaphore tx_shooter_speed{};

namespace
{
bool trig_flag_ready = false;
void SetFrictionTargets(const float speed_first, const float speed_twice)
{
    shooter.fricMotor[0].targetSpeed = static_cast<std::int16_t>(speed_first);
    shooter.fricMotor[1].targetSpeed = static_cast<std::int16_t>(speed_first);
    shooter.fricMotor[2].targetSpeed = static_cast<std::int16_t>(-speed_first);
    shooter.fricMotor[3].targetSpeed = static_cast<std::int16_t>(speed_twice);
    shooter.fricMotor[4].targetSpeed = static_cast<std::int16_t>(speed_twice);
    shooter.fricMotor[5].targetSpeed = static_cast<std::int16_t>(-speed_twice);
}

void Shooter_UpdateRockerControl(void)
{
    if (rcInfo.SWC == 1)
    {
        shooter.fricOpenFlag = true;
        Slope_SetTarget(&shooter.fricSlope1, static_cast<float>(shooter.fricSpd1));
        Slope_SetTarget(&shooter.fricSlope2, static_cast<float>(shooter.fricSpd2));
        Slope_NextVal(&shooter.fricSlope1);
        Slope_NextVal(&shooter.fricSlope2);
        SetFrictionTargets(Slope_GetVal(&shooter.fricSlope1), Slope_GetVal(&shooter.fricSlope2));
    }
    else
    {
        shooter.fricOpenFlag = false;
        Slope_SetTarget(&shooter.fricSlope1, 0.0f);
        Slope_NextVal(&shooter.fricSlope1);
        Slope_SetTarget(&shooter.fricSlope2, 0.0f);
        Slope_NextVal(&shooter.fricSlope2);
        SetFrictionTargets(Slope_GetVal(&shooter.fricSlope1), Slope_GetVal(&shooter.fricSlope2));
    }


    ++shooter.testCount;
    if (trig_flag_ready == false && shooter.fricOpenFlag && rcInfo.SWA == 1)
    {
        trig_flag_ready = true;
        shooter.workState = TRIGGER;
    }
    else if (rcInfo.SWA != 1)
    {
        trig_flag_ready = false;
    }
}

void Heat_Control(void)
{
    static uint16_t shooterNum_last = 0;
    if (shooterNum_last != shooter.shooterNum)
    {
        shooter.BoostHeat.Now_Heat -= shooter.BoostHeat.Consumed_Heat;
        if (shooter.BoostHeat.Now_Heat < shooter.BoostHeat.Limit_Max_Heat)
        {
            shooter.BoostHeat.Now_Heat += shooter.BoostHeat.Cooling_Heat;
        }
        if (shooter.BoostHeat.Now_Heat < shooter.BoostHeat.Limit_Stop_Heat)
        {
            shooter.workState = IDLE;
        }
    }
}
}

namespace JLUni::Shooter_N
{
JLUni::Shooter_N::Shooter_s shooter __attribute__((section(".dtcm_cpp"), aligned(8))) {};
void Shooter_InitPID(void)
{
    for (auto& motor : shooter.fricMotor)
    {
        PID_Clear(&motor.singlePID);
        PID_Init(&motor.singlePID, 40.0f, 0.0f, 5.0f, 10000.0f, 16000.0f);  #elif TRIG_MOTOR_MODE == 1
    }
    #if TRIG_MOTOR_MODE == 0
        DEPID_Clear(&shooter.dm4310_trig.dePID);
        DEPID_Init(&shooter.dm4310_trig.dePID, 180.0f, 0.0f, 1.0f, 15.0f, 20.0f, 0.5f);
    //以下为1Hz测试中供拨盘 26.10.30
    #elif TRIG_MOTOR_MODE == 1
        DEPID_Clear(&shooter.dm4310_trig.dePID);   
        DEPID_Init(&shooter.dm4310_trig.dePID,23.0f, 0.001f, 1.0f, 2.0f, 15.0f, 0.3f);
    #endif
}

void Shooter_Init(void)
{
    shooter = Shooter{};
    shooter.fricOpenFlag = false;
    shooter.fricSpd1 = 3500-800;
    shooter.fricSpd2 = 3500;
    shooter.workState = IDLE;
    shooter.block.judgeCnt = 0;
    shooter.rockerCtrl = false;
    Slope_Init(&shooter.fricSlope1, 5.0f, 0.0f);
    Slope_Init(&shooter.fricSlope2, 5.0f, 0.0f);
    Slope_SetTarget(&shooter.fricSlope1, 0.0f);
    Slope_SetTarget(&shooter.fricSlope2, 0.0f);
    Shooter_InitPID();
}
#if TRIG_MOTOR_MODE == 0
void TrigMotor_SetCalibration(void)
{
    if (shooter.trigCalibrationFlag) 
    {
        return;
    }
    shooter.dm4310_trig.total_target_pos += std::remainder(kTrigStartAngle - shooter.dm4310_trig.pos, kTriggerStep);
    shooter.trigCalibrationFlag = true;
}
#elif TRIG_MOTOR_MODE == 1
float TrigMotor_JudgeAngleRecently(void)
{
    int best_start = 0;
    float best = std::fabs(std::remainder(shooter.dm4310_trig.pos * Trig_Moto2Gear - kTriggerStarts[0], (2.0f * kPi)));
    for (int i = 1; i < 8; ++i)
    {
        float error = std::fabs(std::remainder(shooter.dm4310_trig.pos * Trig_Moto2Gear - kTriggerStarts[i], (2.0f * kPi)));
        if (error < best)
        {
            best = error;
            best_start = i;
        }
    }
    float correction = std::remainder(kTriggerStarts[best_start] - shooter.dm4310_trig.pos * Trig_Moto2Gear, (2.0f * kPi))* Trig_Gear2Moto;
    //拨盘角度误差换算为电机轴角度误差
    return correction ;
}

void TrigMotor_SetCalibration(void)
{
    float value = shooter.dm4310_trig.totalangle + TrigMotor_JudgeAngleRecently();
    shooter.dm4310_trig.total_target_pos = value;
    if (fabs(shooter.dm4310_trig.total_target_pos - shooter.dm4310_trig.totalangle) < 0.1f)
    {
        shooter.dm4310_trig.total_target_pos = 0;
        shooter.dm4310_trig.totalangle = 0;
        trig_calibration_flag = true;
    }

}
#endif

void Shooter_Update(void)
{
    if (rcInfo.SWB == 2) 
    {
        shooter.rockerCtrl = true;
    }
    else if (rcInfo.SWB != 2) 
    {
        shooter.rockerCtrl = false;
    }

    if (shooter.rockerCtrl)
        Shooter_UpdateRockerControl();

    if (shooter.dm4310_trig.vel <= 1.5f && shooter.dm4310_trig.total_target_pos - shooter.dm4310_trig.totalangle >= kTriggerStep && shooter.lastShootMode != TRIGGER_REVERSE)
    {
        ++shooter.block.judgeCnt;
        shooter.testCount = 0U;
        if (shooter.block.judgeCnt >= 150U)
        {
            shooter.block.judgeCnt = 0U;
            shooter.workState = TRIGGER_REVERSE;
        }
    }
    else 
    {
        shooter.block.judgeCnt = 0U;
    }

    if (gimbal.visionEnable)
    {
        ++shooter.visionDelayTime;
        if (shooter.visionDelayTime > 20 && vision_receive.fire != 0 && shooter.fricOpenFlag)
        {
            shooter.workState = TRIGGER;
        }
    }
    else 
    {
        shooter.visionDelayTime = 0;
    }

    Heat_Control();
    switch (shooter.workState)
    {
        case IDLE:
            shooter.lastShootMode = IDLE;
            break;
        case TRIGGER:
            if (shooter.fricOpenFlag)
            {
                shooter.dm4310_trig.total_target_pos += kTriggerStep;
                shooter.lastShootMode = TRIGGER;
                shooter.workState = IDLE;
            }
            break;
        case TRIGGER_REVERSE:
            shooter.dm4310_trig.total_target_pos -= 2.0f * kTriggerStep;
            shooter.workState = IDLE;
            shooter.lastShootMode = TRIGGER_REVERSE;
            break;
        case LOAD:
            break;
    }

    PIDRegulation(&shooter.dm4310_trig.dePID, shooter.dm4310_trig.total_target_pos, shooter.dm4310_trig.totalangle, -shooter.dm4310_trig.vel);
    for (auto& motor : shooter.fricMotor)
    {
        PID_SingleCalc(&motor.singlePID, motor.targetSpeed, motor.speed);
    }
}

void OS_ShooterCallback(void const* argument)
{
    (void)argument;
    while (App_IsInitialized() == 0U) LibXR::Thread::Sleep(1);
    for (;;)
    {
        CAN_RefreshShooterFeedback();
        if (!start_dm3410) 
        {
            TrigMotor_SetCalibration();
        }
        Shooter_Update();
        LibXR::Thread::Sleep(1);
    }
}

}
