/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     pwm_encoder.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    AS5600 PWM输入编码器
  * @version  0.1
  * @date     2021-07-20
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "pwm_encoder.h"

/**
  * @brief    PWM编码器读取值 弧度制
  * 
  */
fp32 pwmEncoder[4];

void PwmEncoder_update(uint8_t i, PwmInput *input) {
  pwmEncoder[i] = (input->duty - AS5600_PWM_MIN_DUTY) / (AS5600_PWM_MAX_DUTY - AS5600_PWM_MIN_DUTY) * 2.0f * PI;
}
