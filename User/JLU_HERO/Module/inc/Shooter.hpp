#pragma once

#include <cstdint>
#include <array>

#include "JLU_HERO/Math/inc/Slope.hpp"
#include "CAN_DJI.hpp"
#include "CAN_DM.hpp"
#include "Shooter.hpp"

namespace  JLUni::Shooter_N
{
    inline static constexpr float Trig_Start_Angle = 1.26282001f;//-0.09f
    #if    (TRIG_MOTOR_MODE == 0)//选择不同的拨盘, 0:侧切拨盘 1：中供拨盘
        inline static constexpr float kTriggerStep = kPi / 3.0f;
    #elif  (TRIG_MOTOR_MODE == 1)
        inline static constexpr float Trig_Gear2Moto = 88.0f/40.0f;
        inline static constexpr float Trig_Moto2Gear = 40.0f/88.0f;//传动比

        inline static constexpr float kTriggerStep = kPi / 4.0f;
        inline static constexpr float kTriggerOffset = - 4.04f;//测出来校准后的其中一个槽位
        //后面有新的结构，应该是用输出力矩的突变来校准，这个暂时不需要
        inline static constexpr float kTriggerStarts[8] = 
        {
            -kPi * 2.0f / 4.0f - kTriggerOffset,
            -kPi * 1.0f / 4.0f - kTriggerOffset,
            -kTriggerOffset,
            kPi * 1.0f / 4.0f - kTriggerOffset,
            kPi * 2.0f / 4.0f - kTriggerOffset,
            kPi * 3.0f / 4.0f - kTriggerOffset,
            kPi * 4.0f / 4.0f - kTriggerOffset,
            kPi * 5.0f / 4.0f - kTriggerOffset
        };
    #endif
}        

namespace JLUni::Shooter_N
{   
    enum ShooterWorkState : std::uint8_t
    {
        IDLE,
        LOAD,
        TRIGGER,
        TRIGGER_REVERSE
    };

    struct ShooterInput//云台从外界获取的输入，关于遥控器和视觉
    {
        Remote_Ctrl_N::rc_state_s remote{};//遥控器输入，对应rcInfo
        bool rockerCtrl{};//是否由遥控器控制
        bool vision_enable  = false;//视觉功能是否使能，以前的vision_enble
        bool vision_receive = false;//视觉是否开火
    };

    struct DM4310_Trig//拨盘电机的运行时变量
    {
        DMmotor dm4310_trig{};//拨盘电机反馈
        ShooterWorkState workState{};//当前送弹状态
        ShooterWorkState lastShootMode{};//上次送弹状态
        struct 
        { 
            std::uint16_t judgeCnt{}; 
        } block;//堵转计数
        bool trig_flag_ready{};//是否已有送弹动作
        bool trigCalibrationFlag{};//拨盘是否已校准好
        std::uint32_t testCount{};
    };

    struct FricMotor
    {
        std::array<DJImotor, 6> fricMotor{};
        Slope fricSlope1{};
        Slope fricSlope2{};
        std::int16_t fricSpd1{};
        std::int16_t fricSpd2{};
        std::int16_t fricSpdOffset{};

        MotorDJI_N::fb_3508_6020_s fricMotor{};//摩擦轮反馈
        JLUni::PIDController_N::PID fricPID{};//摩擦轮pid
        float targetSpeed{};//斜坡的目标转速
        slope_s fricSlope{};//摩擦轮斜坡
        bool fricOpenFlag{};//摩擦轮是否开启
    };

    struct Boost_Heat
    {
        float Now_Heat{300.0f};// 当前热量
        float Limit_Max_Heat{300.0f};// 最大允许热量
        float Limit_Stop_Heat{120.0f};// 低于此值时停止射击
        float Cooling_Heat{0.1f};// 每个控制周期的冷却量
        float Consumed_Heat{100.0f};// 每发射一次消耗的热量
    };

    struct Shooter_S
    {
        ShooterInput input{};//外部输入，包括之前的rcInfo，vision_enble, vision_receive
        DM4310_Trig dm4310_trig{};//拨盘电机参数
        std::array<SingleFricMotor, 6> fricMotor;//摩擦轮电机参数
        std::uint32_t visionDelayTime{};//用来记录视觉连续触发的时间或周期数，防止视觉指令一出现就立即执行射击
        Boost_Heat BoostHeat;
        std::uint16_t shooterNum{};//检测射击次数是否增加，如果增加就扣除一次射击产生的热量
    }

    class Shooter
    {
        public:
            void Shooter_Init(void);
            void Task_Shooter_Callback(void);
        private:
            Shooter_S shooter;
            void TrigMotor_SetCalibration(void);
            void Shooter_InitPID(void);
            void Shooter_RockerCtrl(void);
    };
}
