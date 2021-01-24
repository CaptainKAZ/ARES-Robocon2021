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
/* Definitions for CanCommTask */
osThreadId_t CanCommTaskHandle;
const osThreadAttr_t CanCommTask_attributes = {
  .name = "CanCommTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
/* Definitions for QuickTestTask */
osThreadId_t QuickTestTaskHandle;
const osThreadAttr_t QuickTestTask_attributes = {
  .name = "QuickTestTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for MotionFXTask */
osThreadId_t MotionFXTaskHandle;
const osThreadAttr_t MotionFXTask_attributes = {
  .name = "MotionFXTask",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 1024 * 4
};
/* Definitions for StateSteaming */
osThreadId_t StateSteamingHandle;
const osThreadAttr_t StateSteaming_attributes = {
  .name = "StateSteaming",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void can_communication_task(void *argument);
void quick_test_task(void *argument);
void motionfx_task(void *argument);
void state_steaming_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of CanCommTask */
  CanCommTaskHandle = osThreadNew(can_communication_task, NULL, &CanCommTask_attributes);

  /* creation of QuickTestTask */
  QuickTestTaskHandle = osThreadNew(quick_test_task, NULL, &QuickTestTask_attributes);

  /* creation of MotionFXTask */
  MotionFXTaskHandle = osThreadNew(motionfx_task, NULL, &MotionFXTask_attributes);

  /* creation of StateSteaming */
  StateSteamingHandle = osThreadNew(state_steaming_task, NULL, &StateSteaming_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_can_communication_task */
/**
  * @brief  Function implementing the CANComm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can_communication_task */
__weak void can_communication_task(void *argument)
{
  /* USER CODE BEGIN can_communication_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can_communication_task */
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

/* USER CODE BEGIN Header_motionfx_task */
/**
* @brief Function implementing the MotionFXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motionfx_task */
__weak void motionfx_task(void *argument)
{
  /* USER CODE BEGIN motionfx_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motionfx_task */
}

/* USER CODE BEGIN Header_state_steaming_task */
/**
* @brief Function implementing the StateSteaming thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_state_steaming_task */
__weak void state_steaming_task(void *argument)
{
  /* USER CODE BEGIN state_steaming_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END state_steaming_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
