/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     feedback_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    状态反馈任务
  * @version  0.1
  * @date     2021-03-12
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "feedback_task.h"
#include "string.h"
#include "usart.h"
#include "sbus.h"
static fp32 *  feedback_pointer[FEEDBACK_CHANNEL_NUM];
static uint8_t tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(fp32) + 4];

uint8_t feedback_register(fp32 *ptr, uint8_t channel) {
  if (feedback_pointer[channel] == NULL) {
    feedback_pointer[channel] = ptr;
    return SUCCESS;
  }
  return FAIL;
}

void feedback_task() {
  vTaskDelay(303);
  
  while (1) {
    Sbus_lpf();
    for (uint8_t i = 0; i < FEEDBACK_CHANNEL_NUM; i++) {
      if (feedback_pointer[i] != NULL) {
        memcpy(&tx_buf[i * sizeof(fp32)], feedback_pointer[i], sizeof(fp32));
      }else{
        memset(&tx_buf[i * sizeof(fp32)], 0, sizeof(fp32));
      }
    }
    fp32 time = (fp32)xTaskGetTickCount();
    memcpy(&tx_buf[sizeof(fp32)*FEEDBACK_CHANNEL_NUM], &time, sizeof(fp32));
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(fp32) + 0] = 0x00;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(fp32) + 1] = 0x00;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(fp32) + 2] = 0x80;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(fp32) + 3] = 0x7f;
    HAL_UART_Transmit_DMA(&huart2, tx_buf, sizeof(tx_buf));
    vTaskDelay(1);
  }
}
