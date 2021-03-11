/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     user_it.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    统一处理用户的中断
  * @version  0.1
  * @date     2021-02-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "usart.h"

#include "stopwatch.h"
#include "sbus.h"
#include "imu.h"
#include "imu_comm.h"
#include "INS_task.h"

/**
  * @brief    TIM6中断服务程序，负责计时器更新
  * 
  */
void TIM6_DAC_IRQHandler(void) { stopwatch_hook(); }

/**
  * @brief    USART1中断服务程序，负责SBUS更新
  * 
  */
void USART1_IRQHandler(void) {
  
    sbus_hook();
   
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_DRDY_Pin) {
    if (INS_task_status.imu_initialized) {
      MPU_NSS_L();
      imu_DMA_read();
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI5) {
    MPU_NSS_H();
    HAL_SPI_DMAStop(hspi);
    //唤醒任务
     if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
       static BaseType_t xHigherPriorityTaskWoken;
       vTaskNotifyGiveFromISR(INS_task_handle, &xHigherPriorityTaskWoken);
       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
     }
  }
}
