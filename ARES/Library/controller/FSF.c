/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     FSF.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    实现全状态反馈控制，最基础的控制器
  * @version  0.1
  * @date     2021-02-09
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "FSF.h"

void FSF_ControllerSetParam(Controller *self, ControllerParam *param) {
  if (NULL != self && NULL != param) {
    if (param->type == FSF_CONTROLLER && self->type == FSF_CONTROLLER) {
      self->param = param;
    }
  }
}

fp32 FSF_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out) {
  if (NULL == self && NULL == set && NULL == ref && out == NULL) {
    return 0;
  }
  if (self->type != FSF_CONTROLLER) {
    return 0;
  }

  for (uint8_t i = 0; i < self->I_size; i++) {
    ((FSF_Controller *)self)->err[i] = set[i] - ref[i];
    if (self->constrain->I_loop_Llim[i] < self->constrain->I_loop_Hlim[i]) {
      while (((FSF_Controller *)self)->err[i] <=
             -(self->constrain->I_loop_Hlim[i] - self->constrain->I_loop_Llim[i]) / 2) {
        ((FSF_Controller *)self)->err[i] += self->constrain->I_loop_Hlim[i] - self->constrain->I_loop_Llim[i];
      }
      while (((FSF_Controller *)self)->err[i] >=
             (self->constrain->I_loop_Hlim[i] - self->constrain->I_loop_Llim[i]) / 2) {
        ((FSF_Controller *)self)->err[i] -= self->constrain->I_loop_Hlim[i] - self->constrain->I_loop_Llim[i];
      }
    }
  }

  for (uint8_t i = 0; i < self->O_size;i++){
    out[i] = 0;
    for (uint8_t j = 0; j < self->I_size;j++){
      out[i] += ((FSF_Controller *)self)->err[j] * (((FSF_ControllerParam *)(self->param))->gain + i * self->O_size)[j];
    }
  }

  for (uint8_t i = 0; i < self->O_size;i++){
    CLAMP(out[i], self->constrain->O_constraint[i]);
  }
  
  if(self->O_size==1){
    return out[0];
  }else{
    return 1;
  }
}

void FSF_ControllerInit(FSF_Controller *self, uint8_t I_size, uint8_t O_size, FSF_ControllerParam *param,
                        ControllerConstrain *constrain, fp32 *err) {
  if (NULL != self && NULL != param && NULL != constrain) {
    self->general.type   = FSF_CONTROLLER;
    self->general.I_size = I_size;
    self->general.O_size = O_size;
    if (param->general.type == FSF_CONTROLLER) {
      self->general.param = (ControllerParam *)param;
    }
    self->general.constrain = constrain;
    self->err               = err;
  }
}
