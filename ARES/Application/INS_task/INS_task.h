/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     INS_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    惯性导航任务
  * @version  0.1
  * @date     2020-12-05
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#ifndef INS_TASK_H
#define INS_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "motion_mc.h"

#define MOTIONFX_INIT_TIME 5
#define MFX_DATA_ADDRESS 0x80E0000
#define IMU_CAL_HEAD ARES_HEX
#define IMU_TEMPERATRUE 55

typedef struct{
  uint32_t      head;
  MMC_Output_t mag_cal;
  float        gbias[3];
} imu_cal_t;

typedef struct{
  uint8_t imu_initialized : 1;
  uint8_t imu_calibrated : 1;
} INS_task_status_t;

extern INS_task_status_t INS_task_status;

extern TaskHandle_t INS_task_handle;
extern void INS_task(void *pvParameters);

#endif
