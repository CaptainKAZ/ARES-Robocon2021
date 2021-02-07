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
