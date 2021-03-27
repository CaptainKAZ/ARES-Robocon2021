/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
/* Definitions for QuickTestTask */
osThreadId_t QuickTestTaskHandle;
const osThreadAttr_t QuickTestTask_attributes = {
  .name = "QuickTestTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
/* Definitions for FeedbackTask */
osThreadId_t FeedbackTaskHandle;
const osThreadAttr_t FeedbackTask_attributes = {
  .name = "FeedbackTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for LEDMonitorTask */
osThreadId_t LEDMonitorTaskHandle;
const osThreadAttr_t LEDMonitorTask_attributes = {
  .name = "LEDMonitorTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .priority = (osPriority_t) osPriorityRealtime3,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void motor_task(void *argument);
void quick_test_task(void *argument);
void INS_task(void *argument);
void feedback_task(void *argument);
void led_monitor_task(void *argument);
void chassis_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

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
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor_task, NULL, &MotorTask_attributes);

  /* creation of QuickTestTask */
  QuickTestTaskHandle = osThreadNew(quick_test_task, NULL, &QuickTestTask_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew(INS_task, NULL, &INSTask_attributes);

  /* creation of FeedbackTask */
  FeedbackTaskHandle = osThreadNew(feedback_task, NULL, &FeedbackTask_attributes);

  /* creation of LEDMonitorTask */
  LEDMonitorTaskHandle = osThreadNew(led_monitor_task, NULL, &LEDMonitorTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(chassis_task, NULL, &ChassisTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_motor_task */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_motor_task */
__weak void motor_task(void *argument)
{
  /* USER CODE BEGIN motor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_task */
}

/* USER CODE BEGIN Header_quick_test_task */
/**
* @brief Function implementing the QuickTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_quick_test_task */
__weak void quick_test_task(void *argument)
{
  /* USER CODE BEGIN quick_test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END quick_test_task */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void *argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_feedback_task */
/**
* @brief Function implementing the FeedbackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feedback_task */
__weak void feedback_task(void *argument)
{
  /* USER CODE BEGIN feedback_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END feedback_task */
}

/* USER CODE BEGIN Header_led_monitor_task */
/**
* @brief Function implementing the LEDMonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_monitor_task */
__weak void led_monitor_task(void *argument)
{
  /* USER CODE BEGIN led_monitor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_monitor_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
