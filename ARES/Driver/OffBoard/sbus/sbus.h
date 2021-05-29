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
#define SBUS_MONITOR_TIMEOUT 20

#define SBUS_LX_CHANNEL 3
#define SBUS_LY_CHANNEL 1
#define SBUS_RX_CHANNEL 0
#define SBUS_RY_CHANNEL 2
#define SBUS_V1_CHANNEL 4
#define SBUS_V2_CHANNEL 5
#define SBUS_SA_CHANNEL 6
#define SBUS_SB_CHANNEL 7
#define SBUS_SC_CHANNEL 8
#define SBUS_SD_CHANNEL 9

typedef struct {
  struct {
    fp32     channel[10];
    uint32_t updateTime;
  } real;
  struct {
    fp32     coeff;
    fp32     channel[4];
    uint32_t updateTIme;
  } filtered;
} Sbus;

extern Sbus sbus;

extern void Sbus_init(void);
extern void Sbus_disable(void);
extern void Sbus_restart(void);
extern void Sbus_hook(void);
extern void Sbus_lpf(void);
#endif
