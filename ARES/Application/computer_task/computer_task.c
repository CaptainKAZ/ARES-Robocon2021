/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     computer_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    上位机通信任务
  * @version  0.1
  * @date     2021-04-18
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "computer_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include "ins_task.h"
#include "ops.h"

#define COMPUTER_FRAME_LENGTH 16
#define COMPUTER_RXBUF_SIZE (COMPUTER_FRAME_LENGTH * 2)
#define COMPUTER_TXBUF_SIZE 25
#define COMPUTER_INIT_TIME 1027

static uint8_t opsResetFlag = 0;

typedef struct{
  fp32 ax;
  fp32 ay;
  fp32 wz;
  fp32 yaw;
  fp32 x;
  fp32 y;
} ComputerTxMsg;

static ComputerTxMsg txMsg;
ComputerRxMsg computerRxMsg;
static uint8_t computerRxBuf[2][COMPUTER_RXBUF_SIZE];
static uint8_t computerTxBuf[COMPUTER_TXBUF_SIZE];

uint8_t checkValid(uint8_t *buf, uint8_t len) { 
  uint8_t ret=0;
  for (uint8_t i = 0; i < len;i++){
    ret ^= buf[i];
  }
  return ret;
}

void Computer_rxInit(void) {
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
  }

  //将DMA的外设地址设置为usart6的DR寄存器
  hdma_usart6_rx.Instance->PAR = (uint32_t) & (huart6.Instance->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_usart6_rx.Instance->M0AR = (uint32_t)(&computerRxBuf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_usart6_rx.Instance->M1AR = (uint32_t)(&computerRxBuf[1]);
  //DMA接收长度为缓冲区大小
  hdma_usart6_rx.Instance->NDTR = COMPUTER_RXBUF_SIZE;
  //使能双缓冲区
  SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart6_rx);

  //使能DMA串口接收
  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

  //使能串口空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

/**
  * @brief    上位机协议解析
  * 
  * @param    computer_buf   缓冲区
  */
static void parseComputer(uint8_t *computer_buf) {
  if (computer_buf[0] == '{' && computer_buf[15] == '}') {
    if (checkValid(computer_buf+1,13)==computer_buf[14]){
      computerRxMsg.updateTime = xTaskGetTickCount();
      memcpy(&computerRxMsg.instruct,computer_buf+1,sizeof(ComputerInstruct));
      if(computer_buf[13]){
        opsResetFlag = 1;
      }
    }
  }
}

/**
  * @brief    上位机更新的钩子函数
  * 
  */
void Computer_hook(void) {
  static uint16_t rxLen = 0;

  __HAL_UART_CLEAR_PEFLAG(&huart6);
  __HAL_DMA_DISABLE(&hdma_usart6_rx);
  rxLen                         = COMPUTER_RXBUF_SIZE - hdma_usart6_rx.Instance->NDTR;
  hdma_usart6_rx.Instance->NDTR = COMPUTER_RXBUF_SIZE;

  if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    if (rxLen == COMPUTER_FRAME_LENGTH) {
      parseComputer(computerRxBuf[0]);
    }
  } else {
    hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    if (rxLen == COMPUTER_FRAME_LENGTH) {
      parseComputer(computerRxBuf[1]);
    }
  }
}

/**
  * @brief    上位机串口和DMA重启
  * 
  */
void Computer_rxRestrat(void) {
  __HAL_UART_DISABLE(&huart6);
  __HAL_DMA_DISABLE(&hdma_usart6_rx);

  hdma_usart6_rx.Instance->NDTR = COMPUTER_RXBUF_SIZE;
  __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF3_7);

  __HAL_DMA_ENABLE(&hdma_usart6_rx);
  __HAL_UART_ENABLE(&huart6);
}

void computer_task(void *argument) {
  /* USER CODE BEGIN computer_task */
  vTaskDelay(COMPUTER_INIT_TIME);
  Computer_rxInit();
  /* Infinite loop */
  for (;;) {
    txMsg.ax = motionFX_output.linear_acceleration_9X[0];
    txMsg.ay = motionFX_output.linear_acceleration_9X[1];
    txMsg.wz = DEG2RAD(ops.w);
    txMsg.x  = ops.x * 1000.0f;
    txMsg.y  = ops.y * 1000.0f;
    txMsg.yaw = DEG2RAD(ops.yaw);
    computerTxBuf[0] = '{';
    computerTxBuf[COMPUTER_TXBUF_SIZE - 1] = '}';
    memcpy(computerTxBuf + 1, &txMsg, sizeof(txMsg));
    computerTxBuf[COMPUTER_TXBUF_SIZE - 2] = checkValid(computerTxBuf + 1, sizeof(txMsg));
    HAL_UART_Transmit(&huart6, computerTxBuf, sizeof(computerTxBuf),5);
    if(opsResetFlag){
      opsResetFlag = 0;
      Ops_reset();
    }
    osDelay(100);
  }
  /* USER CODE END computer_task */
}
