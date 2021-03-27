/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     sbus.c
  * @author   ljy
  * @brief    SBUS驱动程序
  * @version  0.1
  * @date     2021-03-26
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

#include "sbus.h"
#include "usart.h"
#include "cmsis_os.h"

#define SBUS_RXBUF_SIZE 50u
#define SBUS_FRAME_LENGTH 25u

#define SBUS_VALUE_MAX ((fp32)671)
#define SBUS_VALUE_OFFSET ((fp32)1024)

Sbus           sbus;
static uint8_t sbusRxBuf[2][SBUS_RXBUF_SIZE];

void Sbus_init(void) {
  //停止DMA接收
  __HAL_DMA_DISABLE(&hdma_usart1_rx);
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
  }

  //将DMA的外设地址设置为USART1的DR寄存器
  hdma_usart1_rx.Instance->PAR = (uint32_t) & (huart1.Instance->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(&sbusRxBuf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_usart1_rx.Instance->M1AR = (uint32_t)(&sbusRxBuf[1]);
  //DMA接收长度为缓冲区大小
  hdma_usart1_rx.Instance->NDTR = SBUS_RXBUF_SIZE;
  //使能双缓冲区
  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart1_rx);

  //使能DMA串口接收
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  //使能串口空闲中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static void parseSbus(volatile const uint8_t *sbusBuf) {
  sbus.channel[0] = (((sbusBuf[1] | (sbusBuf[2] << 8)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[1] = ((((sbusBuf[2] >> 3) | (sbusBuf[3] << 5)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[2] =
      ((((sbusBuf[3] >> 6) | (sbusBuf[4] << 2) | (sbusBuf[5] << 10)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[3] = ((((sbusBuf[5] >> 1) | (sbusBuf[6] << 7)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[4] = ((((sbusBuf[6] >> 4) | (sbusBuf[7] << 4)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[5] =
      ((((sbusBuf[7] >> 7) | (sbusBuf[8] << 1) | (sbusBuf[9] << 9)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[6] = ((((sbusBuf[9] >> 2) | (sbusBuf[10] << 6)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[7] = ((((sbusBuf[10] >> 5) | (sbusBuf[11] << 3)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[8] = (((sbusBuf[12] | (sbusBuf[13] << 8)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  sbus.channel[9] = ((((sbusBuf[13] >> 3) | (sbusBuf[14] << 5)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;

  sbus.updateTime = xTaskGetTickCount();
}

/**
  * @brief    遥控器更新的钩子函数
  * 
  */
void Sbus_hook(void) {
  static uint16_t this_time_rx_len = 0;

  __HAL_UART_CLEAR_PEFLAG(&huart1);
  __HAL_DMA_DISABLE(&hdma_usart1_rx);
  this_time_rx_len              = SBUS_RXBUF_SIZE - hdma_usart1_rx.Instance->NDTR;
  hdma_usart1_rx.Instance->NDTR = SBUS_RXBUF_SIZE;

  if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    if (this_time_rx_len == SBUS_FRAME_LENGTH) {
      parse_sbus(sbusRxBuf[0]);
    }
  } else {
    hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    if (this_time_rx_len == SBUS_FRAME_LENGTH) {
      parse_sbus(sbusRxBuf[1]);
    }
  }
}
