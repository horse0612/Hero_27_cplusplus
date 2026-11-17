

#include "CAN_DJI.hpp"
extern "C"
{

void DJIMotor_StartCalcAngle(DJImotor *motor)
{
    motor->totalAngle = 0;
    motor->lastAngle = motor->angle;
    motor->targetAngle = 0;
}

void DJIMotor_Update(DJImotor *motor, const std::int16_t angle, const std::int16_t speed, const std::int16_t torque, const std::int8_t temperature)
{
    motor->angle = angle;
    motor->speed = speed;
    motor->torque = torque;
    motor->temp = temperature;
}

void DJIMotor_CalcAngle(DJImotor *motor)
{
    int16_t difference = std::remainder( motor->angle - motor->lastAngle, 8192);
    motor->totalAngle += difference;
    motor->lastAngle = motor->angle;
}

void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef *hfdcan,  std::int16_t standardId,  std::int16_t iq1,  std::int16_t iq2,  std::int16_t iq3,  std::int16_t iq4)
{
    std::uint8_t txData[8]{};
    txData[0] = ((iq1 >> 8) & 0xFF);
    txData[1] = (iq1 & 0xFF);
    txData[2] = ((iq2 >> 8) & 0xFF);
    txData[3] = (iq2 & 0xFF);
    txData[4] = ((iq3 >> 8) & 0xFF);
    txData[5] = (iq3 & 0xFF);
    txData[6] = ((iq4 >> 8) & 0xFF);
    txData[7] = (iq4 & 0xFF);
    USER_CAN_Send(hfdcan, standardId, txData);
}

}
