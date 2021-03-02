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
#include "can_comm.h"

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
  if (huart1.Instance->SR & UART_FLAG_IDLE) {
    sbus_hook();
  }
}

/**
  * @brief    GPIO外部中断，主要由MPU6050引发
  * 
  * @param    GPIO_Pin  
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_DRDY_Pin) {
    if (INS_task_status.imu_initialized) {
      MPU_NSS_L();
      imu_DMA_read();
    }
  }
}

/**
  * @brief    SPI_DMA接收完成中断，由IMU信息传输完成引发
  * 
  * @param    hspi      
  */
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  uint8_t           CAN_RxData[8];
  CAN_RxHeaderTypeDef RxMsgHdr;
  CAN_Frame           frame;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsgHdr, CAN_RxData);
  frame.data = CAN_RxData;
  frame.device = hcan->Instance == CAN1 ? INTERNAL_CAN1 : INTERNAL_CAN2;
  frame.id     = RxMsgHdr.IDE == CAN_ID_STD ? RxMsgHdr.StdId : RxMsgHdr.ExtId;
  frame.len    = RxMsgHdr.DLC;
  frame.type   = RxMsgHdr.IDE == CAN_ID_STD ? CAN_FRAME_STD : CAN_FRAME_EXT;
  CAN_RxHook(&frame);
}
