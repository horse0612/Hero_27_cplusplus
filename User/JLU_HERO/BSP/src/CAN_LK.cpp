
#include "CAN_LK.hpp"
extern "C"
{
void STARTRUN(FDCAN_HandleTypeDef *hfdcan, const std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0x88;
    USER_CAN_Send(hfdcan, static_cast<std::int16_t>(standardId), txData);
}

void CLOSEMOTO(FDCAN_HandleTypeDef *hfdcan, const std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0x80;
    USER_CAN_Send(hfdcan, static_cast<std::int16_t>(standardId), txData);
}

void CLEARERROR(FDCAN_HandleTypeDef *hfdcan, const std::uint16_t standardId)
{
    std::uint8_t txData[8]{};
    txData[0] = 0x9B;
    USER_CAN_Send(hfdcan, static_cast<std::int16_t>(standardId), txData);
}

void iqControl(FDCAN_HandleTypeDef *hfdcan, const std::uint16_t standardId, const std::int16_t iqControlValue)
{
    std::uint8_t txData[8]{};
    txData[0] = 0xA1;
    txData[4] = static_cast<std::uint8_t>(iqControlValue & 0xFF);
    txData[5] = static_cast<std::uint8_t>((iqControlValue >> 8) & 0xFF);
    USER_CAN_Send(hfdcan, standardId, txData);
}

void Update9025Motor(LKmoto *motor, std::uint8_t *rxData)
{
    if (rxData[0] == 0xA2U || rxData[0] == 0x9CU || rxData[0] == 0xA1U)
    {
        motor->angle = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rxData[7]) << 8) | rxData[6]);
        motor->speed = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rxData[5]) << 8) | rxData[4]);
        motor->torque = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rxData[3]) << 8) | rxData[2]);
        motor->temp = rxData[1];
    }

    if (rxData[0] == 0x9AU || rxData[0] == 0x9BU)
    {
        motor->temp = rxData[1];
        motor->voltage = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rxData[3]) << 8) | rxData[2]);
        motor->current = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rxData[5]) << 8) | rxData[4]);
        motor->motorstage = rxData[6];
        motor->errorCode = rxData[7];
    }
}
}