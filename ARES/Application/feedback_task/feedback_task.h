/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     feedback_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    状态反馈任务
  * @version  0.1
  * @date     2021-03-12
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef FEEDBACK_TASK
#define FEEDBACK_TASK
#include "cmsis_os.h"
#include "main.h"

#define FEEDBACK_CHANNEL_NUM 10

//在此定义反馈变量通道
#define FEEDBACK_MFX_QUART_1 0
#define FEEDBACK_MFX_QUART_2 1
#define FEEDBACK_MFX_QUART_3 2
#define FEEDBACK_MFX_QUART_4 3
#define FEEDBACK_STEER_ANGLE 4
#define FEEDBACK_STEER_TARGET 5
#define FEEDBACK_DRIVE_TARGET 6
#define FEEDBACK_DRIVE_SPEED  7
extern uint8_t feedback_register(fp32 *ptr, uint8_t channel);

#endif
