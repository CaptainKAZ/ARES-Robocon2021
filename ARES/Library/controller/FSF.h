/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     FSF.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    全状态反馈控制器
  * @version  0.1
  * @date     2021-02-10
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef FSF_H
#define FSF_H
#include "controller.h"

typedef struct {
  Controller general; //继承父类的内容
  fp32 *     err;     //存储误差，需要预先分配
} FSF_Controller;

typedef struct {
  ControllerParam general; //继承父类的内容
  fp32 *         gain;     //增益矩阵[输出][输入]
} FSF_ControllerParam;

extern void FSF_ControllerInit(FSF_Controller *self, uint8_t I_size, uint8_t O_size, FSF_ControllerParam *param,
                               ControllerConstrain *constrain, fp32 *err);

#endif
