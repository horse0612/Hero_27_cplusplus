#pragma once

#include <cstdint>

/*
 * Beep is kept as a small C-compatible interface so that the migrated C++
 * implementation can still be called from CubeMX generated C modules.
 */
enum
{
    T_None = 0,
    T_L1 = 3822,
    T_L2 = 3405,
    T_L3 = 3034,
    T_L4 = 2863,
    T_L5 = 2551,
    T_L6 = 2272,
    T_L7 = 2052,
    T_M1 = 1911,
    T_M2 = 1703,
    T_M3 = 1517,
    T_M4 = 1432,
    T_M5 = 1276,
    T_M6 = 1136,
    T_M7 = 1012,
    T_H1 = 956,
    T_H2 = 851,
    T_H3 = 758,
    T_H4 = 716,
    T_H5 = 638,
    T_H6 = 568,
    T_H7 = 506
};

enum
{
    D_Whole = 2400,
    D_Half = 1200,
    D_Quarter = 600,
    D_Eighth = 300,
    D_Sixteenth = 150
};

struct Note
{
    std::uint16_t period;
    std::uint16_t duration;
};

#ifdef __cplusplus
extern "C"
{
#endif

void Beep_Init(void);
void Beep_Play(std::uint16_t period);
void Beep_PlayOneNote(Note* note);
void Beep_PlayNotes(Note notes[], std::uint8_t note_num);
void Beep_Block_PlayAll(void);
void Task_Beep_Callback(void);
void OS_BeepCallback(void* argument);

#ifdef __cplusplus
}
#endif
