/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     pwm_input.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    PWM输入测量
  * @version  0.1
  * @date     2021-07-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef PWM_INPUT_H
#define PWM_INPUT_H
#include "main.h"
#include "tim.h"

typedef struct {
  uint8_t Cnt;
  uint32_t CCR[2];
  fp32    duty;
  fp32    frequency;
} PwmInput;

extern void PwmInput_captureHook(TIM_HandleTypeDef *htim);
extern void PwmInput_elapsedHook(TIM_HandleTypeDef *htim);

#endif
