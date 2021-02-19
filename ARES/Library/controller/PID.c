/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     PID.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    实现在通用控制器框架下的PID控制
  * @version  0.1
  * @date     2021-02-13
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "PID.h"
#include "user_lib.h"

#define PID ((PID_Controller *)self)
#define PIDPARAM ((PID_ControllerParam *)(((Controller *)self)->param))

#define PID_NEEDINT(self)                                                                                                     \
  ((PIDPARAM->Int_type == CLAMPING_INT) ? ((ABS(PID->out[1]) <= PID->general.constrain->O_constraint[0]) ||                   \
                                           (SIGN(PID->out[1]) ^ (SIGN(PIDPARAM->kI) ^ SIGN(PID->err[0]))))                    \
                                        : 1)

#define PID_Ierr(self)                                                                                                        \
  PIDPARAM->Int_type == BACK_CALCULATION_INT ? PIDPARAM->kB *(PID->out[0] - PID->out[1]) + PID->err[0] : PID->err[0]

void PID_Clear(PID_Controller *self) {
  self->err[0] = self->err[1] = self->err[2] = 0;
  self->Ierr[0] = self->Ierr[1] = 0;
  self->Dout[0] = self->Dout[1] = self->Iout = self->Pout = 0;
  self->out[1] = self->out[0] = 0;
  self->DInt                  = 0;
  PID->dt                     = 0;
  stopwatch_disable(&self->stopwatch);
}

fp32 PID_ControllerUpdate(Controller *self, fp32 *set, fp32 *ref, fp32 *out) {
  (void)out;
  if (NULL == self && NULL == set && NULL == ref) {
    return 0;
  }
  if (self->type != PIDPOS_CONTROLLER && self->type != PIDINC_CONTROLLER) {
    return 0;
  }

  PID->err[2] = PID->err[1];
  PID->err[1] = PID->err[0];
  PID->err[0] = set[0] - ref[0];
  if (self->constrain->I_loop_Llim[0] < self->constrain->I_loop_Hlim[0]) {
    while (PID->err[0] <= -(self->constrain->I_loop_Hlim[0] - self->constrain->I_loop_Llim[0]) / 2) {
      PID->err[0] += self->constrain->I_loop_Hlim[0] - self->constrain->I_loop_Llim[0];
    }
    while (PID->err[0] >= (self->constrain->I_loop_Hlim[0] - self->constrain->I_loop_Llim[0]) / 2) {
      PID->err[0] -= self->constrain->I_loop_Hlim[0] - self->constrain->I_loop_Llim[0];
    }
  }

  PID->Ierr[1] = PID->Ierr[0];
  PID->Ierr[0] = PID_Ierr(self);

  PID->dt = stopwatch_disable(&(PID->stopwatch));
  if (PID->dt > PID->timeout || PID->dt == 0) {
    PID_Clear(PID);
    tic(&(PID->stopwatch));
    return 0;
  }
  tic(&(PID->stopwatch));

  switch (self->type) {
  case PIDPOS_CONTROLLER:
    PID->Pout = PIDPARAM->kP * PID->err[0];
    if (PID_NEEDINT(self)) {
      PID->Iout += PIDPARAM->kI * 0.5 * (PID->Ierr[0] + PID->Ierr[1]) * PID->dt;
      CLAMP(PID->Iout, PIDPARAM->max_Iout);
    }
    if (PIDPARAM->N != 0) {
      PID->Dout[1] = PID->Dout[0];
      PID->Dout[0] = PIDPARAM->kD * (PIDPARAM->N * (PID->err[0] - PID->DInt));
      PID->DInt += 0.5 * PID->dt * (PID->Dout[0] + PID->Dout[1]);
    } else {
      PID->Dout[0] = PIDPARAM->kD * (PID->err[0] - PID->err[1]) / PID->dt;
    }
    PID->out[0] = PID->out[1] = PID->Pout + PID->Iout + PID->Dout[0];

    CLAMP(PID->out[0], self->constrain->O_constraint[0]);
    return PID->out[0];
  case PIDINC_CONTROLLER:
    PID->Pout = PIDPARAM->kP * (PID->err[0] - PID->err[1]);
    PID->Iout = PIDPARAM->kI * (PID->err[0]);
    CLAMP(PID->Iout, PIDPARAM->max_Iout);
    if (PIDPARAM->N == 0) {
      PID->Dout[0] = PIDPARAM->kD * (PID->err[0] - 2 * PID->err[1] + PID->err[2]);
    } else {
      PID->Dout[1] = PID->Dout[0];
      PID->Dout[0] = PIDPARAM->kD * (PIDPARAM->N * (PID->err[0] - PID->err[1] - PID->DInt));
      PID->DInt += 0.5 * PID->dt * (PID->Dout[0] + PID->Dout[1]);
    }
    PID->out[0] += PID->dt * 0.5 * (PID->out[1] + PID->Pout + PID->Iout + PID->Dout[0]);
    PID->out[1] = PID->Pout + PID->Iout + PID->Dout[0];

    CLAMP(PID->out[0], self->constrain->O_constraint[0]);
    return PID->out[0];
  }
}

void PID_ControllerSetParam(Controller *self, ControllerParam *param) {
  if (NULL == self || param == self) {
    return;
  }
  if (param->type == PIDPOS_CONTROLLER || param->type == PIDINC_CONTROLLER) {
    PID_Clear(self);
    self->type  = param->type;
    self->param = param;
  }
}

void PID_ControllerInit(PID_Controller *self, ControllerConstrain *constrain, PID_ControllerParam *param, fp32 timeout) {
  if (NULL == self) {
    return;
  }
  if (param->general.type != PIDPOS_CONTROLLER && param->general.type != PIDINC_CONTROLLER) {
    return;
  }
  self->general.type      = param->general.type;
  self->general.constrain = constrain;
  self->general.param     = param;
  self->general.I_size = self->general.O_size = 1;
  self->timeout                               = timeout;
  stopwatch_register(&self->stopwatch);
  PID_Clear(self);
}
