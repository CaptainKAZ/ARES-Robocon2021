/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     user_init.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    硬件初始化代码，在系统开始前启动
  * @version  0.1
  * @date     2021-03-09
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "main.h"
#include "sbus.h"
#include "tim.h"
#include "can_comm.h"
#include "ops.h"
#include "encoder.h"
#include "gpio.h"
#include "can.h"
#include "mcp2515.h"
#include "monitor_task.h"

void User_Init(void) {
  FLOWLED_OFF(0);
  FLOWLED_OFF(1);
  FLOWLED_OFF(2);
  FLOWLED_OFF(3);
  FLOWLED_OFF(4);
  FLOWLED_OFF(5);
  FLOWLED_OFF(6);
  FLOWLED_OFF(7);
  Sbus_init();
  Ops_init();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR0_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_SET);
  CAN_Start(INTERNAL_CAN1);
  CAN_Start(INTERNAL_CAN2);
  CAN_Start(EXTERNAL_MCP2515);
  HAL_Delay(5);
  HAL_Delay(5);
  for (uint16_t i = 0; i < 20; i++) {
    Encoder_setMode(INTERNAL_CAN1, 0x02, 0xAA);
    Encoder_setMode(INTERNAL_CAN2, 0x02, 0xAA);
    Encoder_setMode(INTERNAL_CAN1, 0x03, 0xAA);
    Encoder_setMode(INTERNAL_CAN2, 0x03, 0xAA);
    Encoder_setFeedbackTime(INTERNAL_CAN1, 0x02, 1000);
    Encoder_setFeedbackTime(INTERNAL_CAN2, 0x02, 1000);
    Encoder_setFeedbackTime(INTERNAL_CAN1, 0x03, 1000);
    Encoder_setFeedbackTime(INTERNAL_CAN2, 0x03, 1000);
  }
}
