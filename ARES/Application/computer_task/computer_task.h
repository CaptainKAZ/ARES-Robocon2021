/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     computer_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    上位机控制任务
  * @version  0.1
  * @date     2021-04-18
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef COMPUTER_TASK_H
#define COMPUTER_TASK_H
#include "main.h"

typedef struct {
  fp32 vx;
  fp32 vy;
  fp32 wz;
} ComputerInstruct;

typedef struct {
  ComputerInstruct instruct;
  uint32_t updateTime;
} ComputerRxMsg;

extern void Computer_hook(void);
extern ComputerRxMsg computerRxMsg;
#endif
