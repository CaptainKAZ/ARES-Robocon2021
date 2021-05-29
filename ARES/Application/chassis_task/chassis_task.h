/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     chassis_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    底盘控制任务
  * @version  0.1
  * @date     2021-04-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "motor.h"
#include "swerve.h"

#define CHASSIS_MAX_TRANS_SPEED 5.5f //单位m/s
#define CHASSIS_MAX_ROT_SPEED 15.0f //单位rad/s

typedef struct{
struct {
  fp32 vx;
  fp32 vy;
  fp32 wz;
} instruct;
struct {
  Motor* motor[2][8];
  Swerve swerve[4];
} component;
} Chassis;

#endif
