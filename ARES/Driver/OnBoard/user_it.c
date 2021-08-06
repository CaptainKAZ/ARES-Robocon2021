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
#include "tim.h"
#include "ops.h"
#include "computer_task.h"
#include "interboard_spi.h"
#include "pwm_input.h"

/**
  * @brief    TIM6中断服务程序，负责计时器更新
  * 
  */
void TIM6_DAC_IRQHandler(void) {
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
      Stopwatch_hook();
    }
  }
}

/**
  * @brief    USART1中断服务程序，负责SBUS更新
  * 
  */
void USART1_IRQHandler(void) {
  if (huart1.Instance->SR & UART_FLAG_RXNE) {
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    return;
  }
  if (huart1.Instance->SR & UART_FLAG_IDLE) {
    Sbus_hook();
  }
}

/**
  * @brief    GPIO外部中断，由MPU6050\MCP2515引发
  * 
  * @param    GPIO_Pin  
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_DRDY_Pin) {
    if (INS_task_status.imu_initialized) {
      MPU_NSS_L();
      imu_DMA_read();
    }
    //  }else if(GPIO_Pin==MCP_INT_Pin){
    //    Mcp2515_rxHook();
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
    //唤醒ins任务
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INS_task_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  } else if (hspi == &hspi1) {
    Interboard_txRxCpltHook();
  }
}
/**
  * @brief    CAN接收完成中断，负责CAN设备更新
  * 
  * @param    hcan      
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  uint8_t             CAN_RxData[8];
  CAN_RxHeaderTypeDef RxMsgHdr;
  CAN_Frame           frame;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsgHdr, CAN_RxData);
  frame.data   = CAN_RxData;
  frame.device = hcan->Instance == CAN1 ? INTERNAL_CAN1 : INTERNAL_CAN2;
  frame.id     = RxMsgHdr.IDE == CAN_ID_STD ? RxMsgHdr.StdId : RxMsgHdr.ExtId;
  frame.len    = RxMsgHdr.DLC;
  frame.type   = RxMsgHdr.IDE == CAN_ID_STD ? CAN_FRAME_STD : CAN_FRAME_EXT;
  CAN_RxHook(&frame);
}

/**
  * @brief    UART7中断，OPS接收
  * 
  */
void UART7_IRQHandler(void) {
  if (huart7.Instance->SR & UART_FLAG_RXNE) {
    __HAL_UART_CLEAR_PEFLAG(&huart7);
    __HAL_UART_FLUSH_DRREGISTER(&huart7);
    return;
  }
  if (huart7.Instance->SR & UART_FLAG_IDLE) {
    Ops_hook();
  }
}

/**
  * @brief    USART3中断，上位机接收
  * 
  */
void USART6_IRQHandler(void) {

  if (huart6.Instance->SR & UART_FLAG_IDLE) {
    Computer_hook();
  } else {
    HAL_UART_IRQHandler(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);
    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    Interboard_errorHook();
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) { PwmInput_captureHook(htim); }
