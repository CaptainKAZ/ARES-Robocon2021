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

#define RM_ECD2RAD 0.0007669903939428206f
#define RM_FRAME_HEAD_1 0x200
#define RM_FRAME_HEAD_2 0x1FF

typedef struct {
  Motor          general;
  uint32_t       rx_timestamp;
  PID_Controller speed_pid;
  PID_Controller angle_pid;
  fp32           set_current;
  fp32           zero;
  uint32_t       cumulative_turn;
  Controller *   alt_controller;
  fp32 (*alt_controller_update)(Motor *motor, Controller *controller);
} RM_Motor;

#endif
