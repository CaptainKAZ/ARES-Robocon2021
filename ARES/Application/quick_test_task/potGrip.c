/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     potGrip.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    控壶
  * @version  0.1
  * @date     2021-07-28
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quick_test_task.h"
#include "motor.h"

typedef struct {
  Motor * gripMotor;
  Motor * xMotor;
  Motor * yMotor;
  fp32    xLim;
  fp32    yLim;
  uint8_t gripZeroed : 1;
  uint8_t xZeroed : 1;
  uint8_t yZeroed : 1;
  uint8_t locked : 1;
} Gripper;

Grip_init(Gripper *gripper, Motor *gripMotor, Motor *xMotor, Motor *yMotor, fp32 xLim, fp32 yLim) {
  gripper->gripMotor = gripMotor;
  gripper->xMotor    = xMotor;
  gripper->yMotor     = yMotor;
  gripper->xLim      = xLim;
  gripper->yLim      = yLim;
}

void Grip_zero() {}

void quick_test_task() { vTaskDelay(1024); }
