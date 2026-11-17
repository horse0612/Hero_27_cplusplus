#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "Judge_crc.hpp"
#include "detect.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "operation.hpp"
#include "uart.hpp"
#include "Shooter.hpp"
namespace LibXR
{
class Semaphore;
}

inline constexpr std::uint8_t JUDGE_DATA_ERROR = 0;
inline constexpr std::uint8_t JUDGE_DATA_CORRECT = 1;
inline constexpr std::uint16_t LEN_HEADER = 5;
inline constexpr std::uint16_t LEN_CMDID = 2;
inline constexpr std::uint16_t LEN_TAIL = 2;
inline constexpr std::uint8_t JUDGE_FRAME_HEADER = 0xA5;
inline constexpr std::size_t JUDGE_MAX_DATA_LENGTH = 300;
inline constexpr std::size_t JUDGE_MAX_FRAME_LENGTH = LEN_HEADER + LEN_CMDID + JUDGE_MAX_DATA_LENGTH + LEN_TAIL;
inline constexpr std::size_t JUDGE_RX_CHUNK_LENGTH = 64;

// 保留原有枚举成员名称，兼容现有调用。
enum RobotColor
{
    RobotColor_Red,
    RobotColor_Blue
};

enum JudgeFrameOffset
{
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7
};

enum FrameHeaderOffset
{
    SOF = 0,
    DATA_LENGTH = 1,
    SEQ = 3,
    CRC8 = 4
};

enum CmdID
{
    ID_game_state = 0x0001,
    ID_game_result = 0x0002,
    ID_game_robot_HP = 0x0003,
    ID_event_data = 0x0101,
    ID_supply_projectile_action = 0x0102,
    ID_referee_warning = 0x0104,
    ID_dart_remaining_time = 0x0105,
    ID_game_robot_state = 0x0201,
    ID_power_heat_data = 0x0202,
    ID_game_robot_pos = 0x0203,
    ID_buff_musk = 0x0204,
    ID_robot_hurt = 0x0206,
    ID_shoot_data = 0x0207,
    ID_bullet_remaining = 0x0208,
    ID_rfid_status = 0x0209,
};

enum JudgeDataLength
{
    LEN_game_state = 11,
    LEN_game_result = 1,
    LEN_game_robot_HP = 32,
    LEN_event_data = 4,
    LEN_supply_projectile_action = 4,
    LEN_referee_warning = 3,
    LEN_dart_remaining_time = 3,
    LEN_game_robot_state = 13,
    LEN_power_heat_data = 16,
    LEN_game_robot_pos = 12,
    LEN_buff_musk = 6,
    LEN_robot_hurt = 1,
    LEN_shoot_data = 7,
    LEN_bullet_remaining = 6,
};

#pragma pack(push, 1)
// 帧头：5 字节。
struct xFrameHeader
{
    std::uint8_t SOF;
    std::uint16_t DataLength;
    std::uint8_t Seq;
    std::uint8_t CRC8;
};

// 0x0001：比赛状态。
struct ext_game_status_t
{
    std::uint8_t game_type : 4;
    std::uint8_t game_progress : 4;
    std::uint16_t stage_remain_time;
    std::uint64_t SyncTimeStamp;
};
// 0x0003：机器人血量。
struct ext_game_robot_HP_t
{
    std::uint16_t red_1_robot_HP;
    std::uint16_t red_2_robot_HP;
    std::uint16_t red_3_robot_HP;
    std::uint16_t red_4_robot_HP;
    std::uint16_t red_5_robot_HP;
    std::uint16_t red_7_robot_HP;
    std::uint16_t red_outpost_HP;
    std::uint16_t red_base_HP;

    std::uint16_t blue_1_robot_HP;
    std::uint16_t blue_2_robot_HP;
    std::uint16_t blue_3_robot_HP;
    std::uint16_t blue_4_robot_HP;
    std::uint16_t blue_5_robot_HP;
    std::uint16_t blue_7_robot_HP;
    std::uint16_t blue_outpost_HP;
    std::uint16_t blue_base_HP;
};

// 0x0101：场地事件。
struct ext_event_data_t
{
    std::uint32_t event_data;
};

// 0x0201：机器人状态。
struct ext_game_robot_status_t
{
    std::uint8_t robot_id;
    std::uint8_t robot_level;
    std::uint16_t current_HP;
    std::uint16_t maximum_HP;
    std::uint16_t shooter_barrel_cooling_value;
    std::uint16_t shooter_barrel_heat_limit;
    std::uint16_t chassis_power_limit;
    std::uint8_t power_management_gimbal_output : 1;
    std::uint8_t power_management_chassis_output : 1;
    std::uint8_t power_management_shooter_output : 1;
};

// 0x0202：功率与热量。
struct ext_power_heat_data_t
{
  uint16_t reserved1; 
  uint16_t reserved2; 
  float reserved3; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
};

// 0x0203：机器人位置。
struct ext_game_robot_pos_t
{
    float x;
    float y;
    float angle;
};

// 0x0204：机器人增益。
struct ext_buff_musk_t
{
    std::uint8_t recovery_buff;
    std::uint8_t cooling_buff;
    std::uint8_t defence_buff;
    std::uint8_t vulnerability_buff;
    std::uint16_t attack_buff;
};

// 0x0207：射击信息。
struct ext_shoot_data_t
{
    std::uint8_t bullet_type;
    std::uint8_t shooter_number;
    std::uint8_t launching_frequency;
    float initial_speed;
};

// 0x0208：弹丸允许发射量。
struct ext_bullet_remaining_t
{
  std::uint16_t projectile_allowance_17mm; 
  std::uint16_t projectile_allowance_42mm;  
  std::uint16_t remaining_gold_coin; 
};

// 交互数据头。
struct ext_student_interactive_header_data_t
{
    std::uint16_t data_cmd_id;
    std::uint16_t send_ID;
    std::uint16_t receiver_ID;
};

struct robot_interactive_data_t
{
    std::uint8_t data[110];
};
#pragma pack(pop)
// 检查关键协议结构的实际长度。
static_assert(sizeof(float) == 4);
static_assert(sizeof(xFrameHeader) == LEN_HEADER);
static_assert(sizeof(ext_game_status_t) == LEN_game_state);
static_assert(sizeof(ext_game_robot_HP_t) == LEN_game_robot_HP);
static_assert(sizeof(ext_event_data_t) == LEN_event_data);
static_assert(sizeof(ext_game_robot_status_t) == LEN_game_robot_state);
static_assert(sizeof(ext_game_robot_pos_t) == LEN_game_robot_pos);
static_assert(sizeof(ext_buff_musk_t) == LEN_buff_musk);
static_assert(sizeof(ext_shoot_data_t) == LEN_shoot_data);
static_assert(sizeof(ext_bullet_remaining_t) == LEN_bullet_remaining);

class JUDGE_Link
{
    public:
    JUDGE_Link();
    void Judge_Start();
    void Arm_Judge_Receive(bool in_isr = false);
    void Judge_Parase();
    LibXR::Semaphore judge_ready;
    private:
    void Judge_Receive(const std::uint8_t* frame, std::size_t frame_length);
    static void Judge_Wait(bool in_isr,JUDGE_Link *self,LibXR::ErrorCode status);
    void ConsumeByte(std::uint8_t byte);
    void ProcessPendingFrames();
    void RemovePendingBytes(std::size_t count);
    LibXR::UART* judge_uart;
    LibXR::Callback<LibXR::ErrorCode> judge_wait_callback;
    LibXR::ReadOperation judge_wait;
    std::array<std::uint8_t, JUDGE_MAX_FRAME_LENGTH> frame_buffer_{};
    std::size_t frame_size_{};
};


extern "C"
{
RobotColor JUDGE_GetSelfColor();
uint8_t JUDGE_GetSelfID();
uint16_t JUDGE_GetClientID();
void JUDGE_Init();
void JUDGE_GetPosition(float *x, float *y);
uint16_t JUDGE_GetChassisPowerLimit();
bool JUDGE_GetShooterOutputState();
bool JUDGE_GetGimbalOutputState();
uint16_t JUDGE_GetHeatLimit();
std::uint16_t JUDGE_GetShootSpeedLimit();
std::uint16_t JUDGE_GetPowerBuffer();
std::int16_t JUDGE_GetRemainHeat();
std::uint16_t JUDGE_GetRemain_42_Num();
std::uint16_t JUDGE_GetHP();
std::uint16_t JUDGE_GetCoolingValue();
void OS_JudgeCallback(void *argument);
}
