#include "CAN_DM.hpp"
std::uint8_t Data_Enable[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
std::uint8_t Data_Failure[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
std::uint8_t Data_Save_zero[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFEU};
DMmotor dmmotor{};

extern "C"
{
bool start_dm3410 = false;
}
extern "C"
{

void DMMotor_StartCalcAngle(DmRxSnapshot *motor,DMmotor *dm_motor)
{
    motor->total_angle = 0.0f;
    dm_motor->last_pos = motor->pos;
}

void dm4310_CalcAngle(DmRxSnapshot *motor,DMmotor *dm_motor)
{
    float difference = std::remainder( motor->pos - dm_motor->last_pos, 2.0f * LibXR::PI);
    motor->total_angle += difference;
    dm_motor->last_pos = motor->pos;
}




int float_to_uint( float value,  float minimum, float maximum,  int bits)
{
    float span = maximum - minimum;
    int maxValue = (1 << bits) - 1;
    return static_cast<int>((value - minimum) * static_cast<float>(maxValue) / span);
}

float uint_to_float( int value,  float minimum,  float maximum,  int bits)
{
    float span = maximum - minimum;
    int maxValue = (1 << bits) - 1;
    return static_cast<float>(value) * span / static_cast<float>(maxValue) + minimum;
}

void PosSpeed_CtrlMotor(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId,  float position,  float velocity)
{
    std::uint8_t *pbuf,*vbuf;
    std::uint8_t txData[8]{};
    pbuf=(uint8_t*)&position;
    vbuf=(uint8_t*)&velocity;

    txData[0] = *pbuf;
    txData[1] = *(pbuf+1);
    txData[2] = *(pbuf+2);
    txData[3] = *(pbuf+3);
    txData[4] = *vbuf;
    txData[5] = *(vbuf+1);
    txData[6] = *(vbuf+2);
    txData[7] = *(vbuf+3);
    USER_CAN_Send(hfdcan, standardId, txData);
}

void Speed_CtrlMotor(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId,  float velocity)
{
    std::uint8_t txData[8]{};
    std::uint8_t *vbuf;
    vbuf=(uint8_t*)&velocity;

    txData[0] = *vbuf;
    txData[1] = *(vbuf+1);
    txData[2] = *(vbuf+2);
    txData[3] = *(vbuf+3);
	txData[4] = 0x00;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;
    USER_CAN_Send(hfdcan, standardId, txData);
}

void MIT_CtrlMotor(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId,  float position,  float velocity,  float kp,  float kd,  float torque)
{
    std::uint16_t positionValue = (float_to_uint(position, P_MIN, P_MAX, 16));
    std::uint16_t velocityValue = (float_to_uint(velocity, V_MIN, V_MAX, 12));
    std::uint16_t kpValue = (float_to_uint(kp, KP_MIN, KP_MAX, 12));
    std::uint16_t kdValue = (float_to_uint(kd, KD_MIN, KD_MAX, 12));
    std::uint16_t torqueValue = (float_to_uint(torque, T_MIN, T_MAX, 12));
    std::uint8_t txData[8]{};
    txData[0] = (positionValue >> 8);
    txData[1] = (positionValue);
    txData[2] = (velocityValue >> 4);
    txData[3] = (((velocityValue & 0x0FU) << 4) | (kpValue >> 8));
    txData[4] = (kpValue);
    txData[5] = (kdValue >> 4);
    txData[6] = (((kdValue & 0x0FU) << 4) | (torqueValue >> 8));
    txData[7] = (torqueValue);
    USER_CAN_Send(hfdcan, standardId, txData);
}

void DMstrat(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFC;
    USER_CAN_Send(hfdcan,standardId, txData);
}

void DMstop(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFD;

    USER_CAN_Send(hfdcan, standardId, txData);
}

void DMerror(FDCAN_HandleTypeDef *hfdcan,  std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFB;
    USER_CAN_Send(hfdcan, standardId, txData);
}
}