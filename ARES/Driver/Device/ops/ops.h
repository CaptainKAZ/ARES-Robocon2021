/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     ops.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    OPS9定位系统驱动
  * @version  0.1
  * @date     2021-03-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef OPS_H
#define OPS_H
#include "main.h"
#include "usart.h"

typedef struct {
  fp32 yaw;
  fp32 pitch;
  fp32 roll;
  fp32 x;
  fp32 y;
  fp32 w;
} Ops;

extern Ops  ops;

extern void Ops_init(void);
extern void Ops_hook(void);

#endif
