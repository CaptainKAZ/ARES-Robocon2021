#include "sbus.h"
#include "usart.h"

fp32           SBUS_CHANNEL[10];
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

void sbus_init(void) {
  //使能DMA串口接收
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
  //使能串口空闲中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  //停止DMA接收
  __HAL_DMA_DISABLE(&hdma_usart1_rx);
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
  }
  //将DMA的外设地址设置为USART1的DR寄存器
  hdma_usart1_rx.Instance->PAR = (uint32_t) & (huart1.Instance->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(&SBUS_rx_buf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_usart1_rx.Instance->M1AR = (uint32_t)(&SBUS_rx_buf[1]);
  //DMA接收长度为缓冲区大小
  hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
  //使能双缓冲区
  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

static void parse_sbus(volatile const uint8_t *sbus_buf) {
  SBUS_CHANNEL[0] = (((sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[1] = ((((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[2] =
      ((((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[3] = ((((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[4] = ((((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[5] =
      ((((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[6] = ((((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[7] = ((((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[8] = (((sbus_buf[12] | (sbus_buf[13] << 8)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
  SBUS_CHANNEL[9] = ((((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07ff) - SBUS_VALUE_OFFSET) / SBUS_VALUE_MAX;
}

void USART1_IRQHandler(void) {
  static uint16_t this_time_rx_len = 0;

  __HAL_UART_CLEAR_PEFLAG(&huart1);
  __HAL_DMA_DISABLE(&hdma_usart1_rx);
  this_time_rx_len              = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
  hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

  if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    if (this_time_rx_len == RC_FRAME_LENGTH) {
      parse_sbus(SBUS_rx_buf[0]);
    }
  } else {
    hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    if (this_time_rx_len == RC_FRAME_LENGTH) {
      parse_sbus(SBUS_rx_buf[1]);
    }
  }
}
