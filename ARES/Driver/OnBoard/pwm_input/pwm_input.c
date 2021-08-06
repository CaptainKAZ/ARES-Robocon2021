/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  * 
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     pwm_input.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    PWM输入测量
  * @version  0.1
  * @date     2021-07-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "pwm_input.h"
#include "pwm_encoder.h"

PwmInput        pwmInput[4];
static uint32_t elapsedCnt;

void PwmInput_elapsedHook(TIM_HandleTypeDef *htim) {
  //暂时不考虑溢出超过一个周期的情况仅作记录
  elapsedCnt++;
  return;
}

void PwmInput_captureHook(TIM_HandleTypeDef *htim) {
  PwmInput *input;
  uint32_t  channel;
  uint8_t   id;
  switch (htim->Channel) {
  case HAL_TIM_ACTIVE_CHANNEL_1:
    input   = &pwmInput[0];
    channel = TIM_CHANNEL_1;
    id      = 0;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_2:
    input   = &pwmInput[1];
    channel = TIM_CHANNEL_2;
    id      = 1;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_3:
    input   = &pwmInput[2];
    channel = TIM_CHANNEL_3;
    id      = 2;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_4:
    input   = &pwmInput[3];
    channel = TIM_CHANNEL_4;
    id      = 3;
    break;
  default: break;
  }
  uint32_t tempCCR;
  switch (input->Cnt) {
  case 0:
    tempCCR = HAL_TIM_ReadCapturedValue(htim, channel);
    if (input->CCR[input->Cnt]) {
      input->frequency = (float)1 / ((tempCCR - input->CCR[0]) * 1e-6);
      input->duty      = (float)(input->CCR[1] - input->CCR[0]) / (tempCCR - input->CCR[0]);
      PwmEncoder_update(id, input);
    }
    input->CCR[input->Cnt] = tempCCR;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    input->Cnt = 1;
    break;
  case 1:
    input->CCR[input->Cnt] = HAL_TIM_ReadCapturedValue(htim, channel);
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
    input->Cnt = 0;
    break;
  }
}
