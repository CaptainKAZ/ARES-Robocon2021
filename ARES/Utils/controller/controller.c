/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     controller.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    统一管理控制器类的虚函数
  * @version  0.1
  * @date     2021-02-19
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

#include "controller.h"

extern void FSF_ControllerSetParam(Controller *self, ControllerParam *param);
extern fp32 FSF_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out);
extern void PID_ControllerSetParam(Controller *self, ControllerParam *param);
extern fp32 PID_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out);
extern void ADRC_ControllerSetParam(Controller *self, ControllerParam *param);
extern fp32 ADRC_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out);

static void (*setParamTable[CONTROLLER_TYPE_NUM])(Controller *self, ControllerParam *param) = {
    NULL, FSF_ControllerSetParam, PID_ControllerSetParam, PID_ControllerSetParam, ADRC_ControllerSetParam};

static fp32 (*updateTable[CONTROLLER_TYPE_NUM])(Controller *self, fp32 *set, fp32 *ref, fp32 *out) = {
    NULL, FSF_ControllerUpdate, PID_ControllerUpdate, PID_ControllerUpdate, ADRC_ControllerUpdate};

/**
  * @brief    控制器更改参数函数
  * 
  * @param    self      控制器结构体
  * @param    param     控制器参数结构体
  */
void controllerSetParam(Controller *self, ControllerParam *param) { setParamTable[self->type](self, param); }
/**
  * @brief    控制器更新函数
  * 
  * @param    self      控制器结构体
  * @param    set       期望值
  * @param    ref       反馈值
  * @param    out       当输出大小不为1的时候需要传入输出指针
  * @return   fp32      输出大小为1时控制器计算返回值即为输出大小,否则为0
  */
fp32 controllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out) {
  return updateTable[self->type](self, set, ref, out);
}
