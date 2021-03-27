/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     sbus.h
  * @author   ljy
  * @brief    
  * @version  0.1
  * @date     2021-03-26
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef SBUS_H
#define SBUS_H

#include "main.h"

typedef struct {
  fp32 channel[10];
  uint32_t updateTime;
} Sbus;

extern Sbus sbus;

extern void Sbus_init(void);
extern void Sbus_disable(void);
extern void Sbus_restart(void);
extern void Sbus_hook(void);
#endif
