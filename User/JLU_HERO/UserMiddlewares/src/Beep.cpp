#include "Beep.hpp"
#include "MyQueue.hpp"
#include "ErrorTask.hpp"
#include "libxr.hpp"
extern "C"
{
#include "app_main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "tim.h"
}

namespace
{
constexpr std::uint16_t kMaxNoteQueueSize = 50;
Queue note_queue{};
Note note_queue_buffer[kMaxNoteQueueSize]{};

void SoftwareDelay(const std::uint16_t duration_ms)
{
    volatile std::uint32_t count = static_cast<std::uint32_t>(duration_ms) * 12000;
    while (count != 0)
    {
        count = count - 1;
    }
}
}  

extern "C" void Beep_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Queue_Init(&note_queue, kMaxNoteQueueSize);
    Queue_AttachBuffer(&note_queue, note_queue_buffer, sizeof(Note));
}

extern "C" void Beep_Play(const std::uint16_t period)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SetAutoreload(&htim12, period);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, period / 2);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    
}

extern "C" void Beep_PlayOneNote(Note* note)
{
    Queue_Enqueue(&note_queue, note);
    (void)xTaskNotifyGive(reinterpret_cast<TaskHandle_t>(BeepTaskHandle));
}

extern "C" void Beep_PlayNotes(Note notes[], const std::uint8_t note_num)
{
    for (std::uint8_t index = 0; index < note_num; ++index)
    {
        Queue_Enqueue(&note_queue, &notes[index]);
    }
    (void)xTaskNotifyGive(reinterpret_cast<TaskHandle_t>(BeepTaskHandle));
}

extern "C" void Beep_Block_PlayAll(void)
{
    while (Queue_IsEmpty(&note_queue) == 0)
    {
        auto* note = static_cast<Note*>(Queue_Dequeue(&note_queue));
        Beep_Play(note->period);
        SoftwareDelay(note->duration);
    }
    Beep_Play(T_None);
}

extern "C" void Task_Beep_Callback(void)
{
    if(Queue_IsEmpty(&note_queue))
    {
        Beep_Play(T_None);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    else
    {
        auto* note = static_cast<Note*>(Queue_Dequeue(&note_queue));
        Beep_Play(note->period);
        osDelay(note->duration);
    }
}

extern "C" void OS_BeepCallback(void* argument)
{
    (void)argument;
  
    Note startup_melody[] = {{T_None, D_Sixteenth},{T_None, D_Sixteenth},{T_M1,D_Sixteenth},{T_M3,D_Sixteenth},{T_M5,D_Sixteenth}};
    Beep_PlayNotes(startup_melody,2);
    for (;;)
    {
        Task_Beep_Callback();
        osDelay(20);
    }
}
