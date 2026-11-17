/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ErrorTask */
osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RS485Task */
osThreadId_t RS485TaskHandle;
const osThreadAttr_t RS485Task_attributes = {
  .name = "RS485Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ShooterTask */
osThreadId_t ShooterTaskHandle;
const osThreadAttr_t ShooterTask_attributes = {
  .name = "ShooterTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for VisionTask */
osThreadId_t VisionTaskHandle;
const osThreadAttr_t VisionTask_attributes = {
  .name = "VisionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for JudgeTask */
osThreadId_t JudgeTaskHandle;
const osThreadAttr_t JudgeTask_attributes = {
  .name = "JudgeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DetectTask */
osThreadId_t DetectTaskHandle;
const osThreadAttr_t DetectTask_attributes = {
  .name = "DetectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for BeepTask */
osThreadId_t BeepTaskHandle;
const osThreadAttr_t BeepTask_attributes = {
  .name = "BeepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RcTask */
osThreadId_t RcTaskHandle;
const osThreadAttr_t RcTask_attributes = {
  .name = "RcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TaskStart(void *argument);
void OS_ErrorCallback(void *argument);
void OS_Board2BoardCallback(void *argument);
void OS_IMUCallback(void *argument);
void OS_GimbalCallback(void *argument);
void OS_ShooterCallback(void *argument);
void OS_VisionCallback(void *argument);
void OS_JudgeCallback(void *argument);
void OS_DetectCallback(void *argument);
void OS_BeepCallback(void *argument);
void OS_RcCallback(void *argument);
void OS_MotorCallback(void *argument);
void OS_LedCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(TaskStart, NULL, &defaultTask_attributes);

  /* creation of ErrorTask */
  ErrorTaskHandle = osThreadNew(OS_ErrorCallback, NULL, &ErrorTask_attributes);

  /* creation of RS485Task */
  RS485TaskHandle = osThreadNew(OS_Board2BoardCallback, NULL, &RS485Task_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(OS_IMUCallback, NULL, &IMUTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(OS_GimbalCallback, NULL, &GimbalTask_attributes);

  /* creation of ShooterTask */
  ShooterTaskHandle = osThreadNew(OS_ShooterCallback, NULL, &ShooterTask_attributes);

  /* creation of VisionTask */
  VisionTaskHandle = osThreadNew(OS_VisionCallback, NULL, &VisionTask_attributes);

  /* creation of JudgeTask */
  JudgeTaskHandle = osThreadNew(OS_JudgeCallback, NULL, &JudgeTask_attributes);

  /* creation of DetectTask */
  DetectTaskHandle = osThreadNew(OS_DetectCallback, NULL, &DetectTask_attributes);

  /* creation of BeepTask */
  BeepTaskHandle = osThreadNew(OS_BeepCallback, NULL, &BeepTask_attributes);

  /* creation of RcTask */
  RcTaskHandle = osThreadNew(OS_RcCallback, NULL, &RcTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(OS_MotorCallback, NULL, &MotorTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(OS_LedCallback, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TaskStart */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskStart */
__weak void TaskStart(void *argument)
{
  /* USER CODE BEGIN TaskStart */
  app_main();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskStart */
}

/* USER CODE BEGIN Header_OS_ErrorCallback */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ErrorCallback */
__weak void OS_ErrorCallback(void *argument)
{
  /* USER CODE BEGIN OS_ErrorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ErrorCallback */
}

/* USER CODE BEGIN Header_OS_Board2BoardCallback */
/**
* @brief Function implementing the RS485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_Board2BoardCallback */
__weak void OS_Board2BoardCallback(void *argument)
{
  /* USER CODE BEGIN OS_Board2BoardCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_Board2BoardCallback */
}

/* USER CODE BEGIN Header_OS_IMUCallback */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_IMUCallback */
__weak void OS_IMUCallback(void *argument)
{
  /* USER CODE BEGIN OS_IMUCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_IMUCallback */
}

/* USER CODE BEGIN Header_OS_GimbalCallback */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_GimbalCallback */
__weak void OS_GimbalCallback(void *argument)
{
  /* USER CODE BEGIN OS_GimbalCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_GimbalCallback */
}

/* USER CODE BEGIN Header_OS_ShooterCallback */
/**
* @brief Function implementing the ShooterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ShooterCallback */
__weak void OS_ShooterCallback(void *argument)
{
  /* USER CODE BEGIN OS_ShooterCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ShooterCallback */
}

/* USER CODE BEGIN Header_OS_VisionCallback */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_VisionCallback */
__weak void OS_VisionCallback(void *argument)
{
  /* USER CODE BEGIN OS_VisionCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_VisionCallback */
}

/* USER CODE BEGIN Header_OS_JudgeCallback */
/**
* @brief Function implementing the JudgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_JudgeCallback */
__weak void OS_JudgeCallback(void *argument)
{
  /* USER CODE BEGIN OS_JudgeCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_JudgeCallback */
}

/* USER CODE BEGIN Header_OS_DetectCallback */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_DetectCallback */
__weak void OS_DetectCallback(void *argument)
{
  /* USER CODE BEGIN OS_DetectCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_DetectCallback */
}

/* USER CODE BEGIN Header_OS_BeepCallback */
/**
* @brief Function implementing the BeepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_BeepCallback */
__weak void OS_BeepCallback(void *argument)
{
  /* USER CODE BEGIN OS_BeepCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_BeepCallback */
}

/* USER CODE BEGIN Header_OS_RcCallback */
/**
* @brief Function implementing the RcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_RcCallback */
__weak void OS_RcCallback(void *argument)
{
  /* USER CODE BEGIN OS_RcCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_RcCallback */
}

/* USER CODE BEGIN Header_OS_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_MotorCallback */
__weak void OS_MotorCallback(void *argument)
{
  /* USER CODE BEGIN OS_MotorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_MotorCallback */
}

/* USER CODE BEGIN Header_OS_LedCallback */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_LedCallback */
__weak void OS_LedCallback(void *argument)
{
  /* USER CODE BEGIN OS_LedCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_LedCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

