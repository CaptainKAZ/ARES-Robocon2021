/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     dr_chassis.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    DR底盘控制程序
  * @version  0.1
  * @date     2021-07-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef DR_CHASSIS_H
#define DR_CHASSIS_H
#include "main.h"
#include "fsm.h"
#include "motor.h"

#define WHEEL_CENTER_DISTANCE_NARROW 0.46227697325304880472162984249375f
#define WHEEL_CENTER_DISTANCE_EXPAND 2.6161230857893517698111141784248f
#define WHEEL_RADIUS 0.076f
#ifndef SQRT2
#define SQRT2 1.4142135623730950488016887242097f
#endif

typedef enum {
  CHASSIS_FEEDBACK_WHEEL,
  CHASSIS_FEEDBACK_FK,
  CHASSIS_FEEDBACK_OPS_YAW,
  CHASSIS_FEEDBACK_OPS_FULL,
  CHASSIS_FEEDBACK_GYRO,
} ChassisFeedbackMode;

typedef struct {
  struct {
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 expand;
    fp32 expandForce; //给定的恒力
  } set;
  struct {
    fp32 wheelSpeed[4];
    struct {
      fp32 x;
      fp32 y;
      fp32 z;
      fp32 expand;
    } a;
    struct {
      fp32 x;
      fp32 y;
      fp32 z;
      fp32 expand;
    } v;
    struct {
      fp32 x;
      fp32 y;
      fp32 z;
      fp32 expand;
    } p;
    uint32_t updateTime;
  } feedback;

  fp32 out[4];

  ChassisFeedbackMode feedbackMode;
  Motor *             motor[8];
  StateMachine              chassisFsm;
  fp32                wheelCenterDistance;
} DrChassis;

#endif
