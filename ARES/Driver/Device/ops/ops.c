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
#include "string.h"

uint8_t ops_rx_buf[2][OPS_RXBUF_SIZE];
OPSData OPS_DATA;

void ops_init(void) {
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_uart7_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_uart7_rx);
  }
  //将DMA的外设地址设置为UART7的DR寄存器
  hdma_uart7_rx.Instance->PAR = (uint32_t) & (huart7.Instance->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_uart7_rx.Instance->M0AR = (uint32_t)(&ops_rx_buf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_uart7_rx.Instance->M1AR = (uint32_t)(&ops_rx_buf[1]);
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
}

static void parse_ops(volatile const uint8_t *ops_buf) {
  if (ops_buf[0] == 0x0D && ops_buf[1] == 0x0A && ops_buf[26] == 0x0A && ops_buf[27] == 0x0D) {
    memcpy((uint8_t *)&OPS_DATA,(void*)&ops_buf[2],sizeof(OPSData));
  }
}

/**
  * @brief    OPS更新的钩子函数
  * 
  */
void ops_hook(void) {
  static uint16_t this_time_rx_len = 0;

  __HAL_UART_CLEAR_PEFLAG(&huart7);
  __HAL_DMA_DISABLE(&hdma_uart7_rx);
  this_time_rx_len             = OPS_RXBUF_SIZE - hdma_uart7_rx.Instance->NDTR;
  hdma_uart7_rx.Instance->NDTR = OPS_RXBUF_SIZE;

  if ((hdma_uart7_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    hdma_uart7_rx.Instance->CR |= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_uart7_rx);
    if (this_time_rx_len == OPS_FRAME_LENGTH) {
      parse_ops(ops_rx_buf[0]);
    }
  } else {
    hdma_uart7_rx.Instance->CR &= ~(DMA_SxCR_CT);
    __HAL_DMA_ENABLE(&hdma_uart7_rx);
    if (this_time_rx_len == OPS_FRAME_LENGTH) {
      parse_ops(ops_rx_buf[1]);
    }
  }
}
