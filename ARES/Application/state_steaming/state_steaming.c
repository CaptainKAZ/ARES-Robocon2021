/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     state_steaming.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    流式反馈数据任务
  * @version  0.1
  * @date     2021-01-23
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "main.h"
#include "state_steaming.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "user_lib.h"
#include <stdio.h>
#include "usart.h"

static uint8_t               message_buffer[MSG_BUF_SIZE];
static StaticMessageBuffer_t msg_buf_struct;
static MessageBufferHandle_t msg_buf_handle;
int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1);
  return (ch);  
}

/*
void state_steaming_task(void *argument) {
  msg_buf_handle=xMessageBufferCreateStatic(MSG_BUF_SIZE, message_buffer, &msg_buf_struct);
  vTaskDelay(STATE_STEAMING_INIT_TIME);
  for (;;) {
    uint8_t msg_receive[PACK_SIZE];
    xMessageBufferReceive(msg_buf_handle, msg_receive, 20, portMAX_DELAY);
    HAL_UART_Transmit_DMA(&huart2, msg_receive, PACK_SIZE);
  }
}
*/
