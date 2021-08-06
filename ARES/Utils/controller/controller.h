/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     controller.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    控制器父类，主要定义抽象的控制器
  * @version  0.1
  * @date     2021-02-09
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "main.h"

typedef enum {
  NONE_CONTROLLER = 0, //未初始化
  FSF_CONTROLLER,      //全状态反馈控制
  PIDPOS_CONTROLLER,   //位置式PID控制
  PIDINC_CONTROLLER,   //增量式PID控制
  ADRC_CONTROLLER,     //ADRC控制器
  CONTROLLER_TYPE_NUM,
} ControllerType;

typedef struct {
  ControllerType type;
} ControllerParam;

typedef struct {
  fp32 *O_Hlim;      //输出限制(0即为不限制)
  fp32 *O_Llim;      //输出限制(0即为不限制)
  fp32 *I_loop_Llim; //循环输入的下限(上下限设置为0即为不限制)
  fp32 *I_loop_Hlim; //循环输入的上限(上下限设置为0即为不限制)
} ControllerConstrain;

typedef struct Controller {
  ControllerType       type;
  uint8_t              I_size;    //输入的数据大小
  uint8_t              O_size;    //输出的数据大小
  ControllerConstrain *constrain; //控制器限制
  ControllerParam *    param;     //控制器参数
} Controller;

#define CLAMP(input, min, max)                                                                                                \
  {                                                                                                                           \
    if ((input) > (max)) {                                                                                                    \
      (input) = (max);                                                                                                        \
    } else if ((input) < (min)) {                                                                                             \
      (input) = (min);                                                                                                        \
    }                                                                                                                         \
  }

extern void controllerSetParam(Controller *self, ControllerParam *param);
extern fp32 controllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out);

#endif
