/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     RM_motor.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    RM电机
  * @version  0.1
  * @date     2021-02-22
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef RM_MOTOR_H
#define RM_MOTOR_H
#include "main.h"
#include "motor.h"
#include "PID.h"

#define RM_MOTOR_ECD2RAD 7.6699039394282061485904379474597e-4f
#define RM_MOTOR_FRAME_HEAD_1 0x200
#define RM_MOTOR_FRAME_HEAD_2 0x1FF
#define RM_MOTOR_MONITOR_TIMEOUT 10

#define M3508_REDUCTION_RATIO 19.20320855614973f
#define M2006_REDUCTION_RATIO 36.0f

typedef struct {
  Motor          general;
  uint32_t       rx_timestamp;
  PID_Controller speed_pid;
  PID_Controller angle_pid;
  fp32           set_current;
} RM_Motor;

#endif
