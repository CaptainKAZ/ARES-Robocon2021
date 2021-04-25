/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     ops.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    OPS9定位系统驱动
  * @version  0.1
  * @date     2021-03-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "ops.h"
#include <string.h>
#include <math.h>
#include "monitor_task.h"

#define OPS_RXBUF_SIZE 56u
#define OPS_FRAME_LENGTH 28u

uint8_t     opsRxBuf[2][OPS_RXBUF_SIZE];
Ops         ops;
MonitorList opsMonitor;

void Ops_guard(uint32_t *errorReg, uint32_t *errorTime);

/**
  * @brief    OPS初始化
  * 
  */
void Ops_init(void) {
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_uart7_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_uart7_rx);
  }

  //将DMA的外设地址设置为UART7的DR寄存器
  hdma_uart7_rx.Instance->PAR = (uint32_t) & (huart7.Instance->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_uart7_rx.Instance->M0AR = (uint32_t)(&opsRxBuf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_uart7_rx.Instance->M1AR = (uint32_t)(&opsRxBuf[1]);
  //DMA接收长度为缓冲区大小
  hdma_uart7_rx.Instance->NDTR = OPS_RXBUF_SIZE;
  //使能双缓冲区
  SET_BIT(hdma_uart7_rx.Instance->CR, DMA_SxCR_DBM);
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_uart7_rx);

  //使能DMA串口接收
  SET_BIT(huart7.Instance->CR3, USART_CR3_DMAR);

  //使能串口空闲中断
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);

  Monitor_registor(&opsMonitor, Ops_guard, MONITOR_OPS_ID);
}

/**
  * @brief    OPS协议解析
  * 
  * @param    ops_buf   缓冲区
  */
static void parseOps(volatile const uint8_t *ops_buf) {
  if (ops_buf[0] == 0x0D && ops_buf[1] == 0x0A && ops_buf[26] == 0x0A && ops_buf[27] == 0x0D) {
    memcpy((uint8_t *)&ops, (void *)&ops_buf[2], sizeof(Ops) - 4);
    ops.updateTime = xTaskGetTickCount();
  }
}

/**
  * @brief    OPS更新的钩子函数
  * 
  */
void Ops_hook(void) {
  static uint16_t rxLen = 0;

  __HAL_UART_CLEAR_PEFLAG(&huart7);
  __HAL_DMA_DISABLE(&hdma_uart7_rx);
  rxLen                        = OPS_RXBUF_SIZE - hdma_uart7_rx.Instance->NDTR;
  hdma_uart7_rx.Instance->NDTR = OPS_RXBUF_SIZE;

  if ((hdma_uart7_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    hdma_uart7_rx.Instance->CR |= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_uart7_rx);
    if (rxLen == OPS_FRAME_LENGTH) {
      parseOps(opsRxBuf[0]);
    }
  } else {
    hdma_uart7_rx.Instance->CR &= ~(DMA_SxCR_CT);
    __HAL_DMA_ENABLE(&hdma_uart7_rx);
    if (rxLen == OPS_FRAME_LENGTH) {
      parseOps(opsRxBuf[1]);
    }
  }
}

/**
  * @brief    OPS串口和DMA重启
  * 
  */
void Ops_restrat(void) {
  __HAL_UART_DISABLE(&huart7);
  __HAL_DMA_DISABLE(&hdma_uart7_rx);

  hdma_uart7_rx.Instance->NDTR = OPS_RXBUF_SIZE;
  __HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_RXNE);
  __HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx, DMA_FLAG_TCIF3_7);

  __HAL_DMA_ENABLE(&hdma_uart7_rx);
  __HAL_UART_ENABLE(&huart7);
}

/**
  * @brief    OPS守护函数
  * 
  * @param    errorReg  错误寄存器
  * @param    errorTime 错误时间
  */
void Ops_guard(uint32_t *errorReg, uint32_t *errorTime) {
  *errorReg  = 0;
  *errorTime = 0;
  if (xTaskGetTickCount() - ops.updateTime > OPS_MONITOR_TIMEOUT) {
    SET_BIT(*errorReg, MONITOR_ERROR_EXIST);
    SET_BIT(*errorReg, MONITOR_ERROR_LOST);
    Ops_restrat();
  }
  //TODO: Better judge condition
  if ((isnormal(ops.w) && isnormal(ops.x) && isnormal(ops.y) && isnormal(ops.yaw) && isnormal(ops.pitch) &&
       isnormal(ops.roll))) {
    SET_BIT(*errorReg, MONITOR_ERROR_EXIST);
    SET_BIT(*errorReg, MONITOR_ERROR_INVALID);
    memset(&ops, 0, sizeof(ops));
  }
  if (errorReg) {
    *errorTime = xTaskGetTickCount();
  }
}

void Ops_reset(void) {
  uint8_t txBuf[4];
  txBuf[0] = 'A';
  txBuf[1] = 'C';
  txBuf[2] = 'T';
  txBuf[3] = '0';
  HAL_UART_Transmit(&huart7, txBuf, 4, 1);
}
