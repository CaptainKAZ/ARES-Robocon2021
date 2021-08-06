/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     pwm_encoder.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    AS5600 PWM输入编码器
  * @version  0.1
  * @date     2021-07-20
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef PWM_ENCODER
#define PWM_ENCODER
#include "main.h"
#include "pwm_input.h"

#define AS5600_PWM_MIN_DUTY 0.02941852447713169386347965984831f
#define AS5600_PWM_MAX_DUTY 0.97058147552286830613652034015169f

extern void PwmEncoder_update(uint8_t i, PwmInput* input);
extern fp32 pwmEncoder[4];
#endif
