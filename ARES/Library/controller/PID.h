/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     PID.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    实现在通用控制器框架下的PID控制
  * @version  0.1
  * @date     2021-02-13
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef PID_H
#define PID
#include "controller.h"
#include "stopwatch.h"

typedef enum {
  NORMAL,
  CLAMPING_INT,
  BACK_CALCULATION_INT,
} PID_IntegratorType;

typedef struct {
  ControllerParam    general;
  fp32               kP;
  fp32               kI;
  fp32               kD;
  fp32               kB;       //反馈抗饱和系数
  fp32               max_Iout; //积分器钳位
  PID_IntegratorType Int_type; //积分器形式
  fp32               N;        //一阶滤波常数
} PID_ControllerParam;

typedef struct {
  Controller  general; //继承控制器父类
  fp32        timeout; //积分器超时
  stopwatch_t stopwatch;
  fp32        dt;
  fp32        Pout;
  fp32        Iout;
  fp32        Dout[2];
  fp32        DInt;
  fp32        out[2];
  fp32        err[3];
  fp32        Ierr[2];
} PID_Controller;

extern void PID_ControllerInit(PID_Controller *self, ControllerConstrain *constrain, PID_ControllerParam *param, fp32 timeout);

#endif
