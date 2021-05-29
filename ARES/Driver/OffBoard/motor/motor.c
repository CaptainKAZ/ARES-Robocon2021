/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     motor.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    电机统一API接口
  * @version  0.1
  * @date     2021-03-03
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "motor.h"
#include "vesc_motor.h"
#include "rm_motor.h"
#include "rmd_motor.h"

extern void RM_Motor_setSpeedPid(Motor *self, PID_ControllerParam *param);
extern void RMD_Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param);

extern void RM_Motor_setAnglePid(Motor *self, PID_ControllerParam *param);
extern void RMD_Motor_SetAnglePID(Motor *self, PID_ControllerParam *param);

extern void RMD_Motor_Zero(Motor *self);
extern void RM_Motor_zero(Motor *self);
extern void VESC_Motor_Zero(Motor *self);

extern void RM_Motor_setCurrent(Motor *self, fp32 mA, uint32_t timeout);
extern void RMD_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);
extern void VESC_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);

extern void RM_Motor_setSpeed(Motor *self, fp32 rpm, uint32_t timeout);
extern void VESC_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);
extern void RMD_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);

extern void RM_Motor_setAngle(Motor *self, fp32 rad, uint32_t timeout);
extern void RMD_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);
extern void VESC_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);

extern void RM_Motor_setAltController(Motor *self, Controller *alt_controller, void *param,
                                      MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                                 void *param));
extern void RMD_Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                                       MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                                  void *param));
extern void VESC_Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                                        MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                                   void *param));

extern void RM_Motor_altControl(Motor *self, uint32_t timeout);
extern void RMD_Motor_AltControl(Motor *self, uint32_t timeout);
extern void VESC_Motor_AltControl(Motor *self, uint32_t timeout);

extern Motor *RM_Motor_get(CAN_Device device, uint8_t id);
extern Motor *RMD_Motor_Find(CAN_Device device, uint8_t id);
extern Motor *VESC_Motor_Find(CAN_Device device, uint8_t id);

void (*SetSpeedPIDTable[])(Motor *self, PID_ControllerParam *param) = {NULL, RM_Motor_setSpeedPid, NULL,
                                                                       RMD_Motor_SetSpeedPID};
void (*SetAnglePIDTable[])(Motor *self, PID_ControllerParam *param) = {NULL, RM_Motor_setAnglePid, NULL,
                                                                       RMD_Motor_SetAnglePID};
void (*ZeroTable[])(Motor *self)                                    = {NULL, RM_Motor_zero, VESC_Motor_Zero, RMD_Motor_Zero};
void (*SetCurrentTable[])(Motor *self, fp32 current, uint32_t timeout) = {NULL, RM_Motor_setCurrent, VESC_Motor_SetCurrent,
                                                                          RMD_Motor_SetCurrent};
void (*SetSpeedTable[])(Motor *self, fp32 rpm, uint32_t timeout)       = {NULL, RM_Motor_setSpeed, VESC_Motor_SetSpeed,
                                                                    RMD_Motor_SetSpeed};
void (*SetAngleTable[])(Motor *self, fp32 rad, uint32_t timeout)       = {NULL, RM_Motor_setAngle, VESC_Motor_SetAngle,
                                                                    RMD_Motor_SetAngle};
void (*SetAltControllerTable[])(Motor *self, Controller *alt_controller, void *param,
                                MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                           void *param)) = {
    NULL, RM_Motor_setAltController, VESC_Motor_SetAltController, RMD_Motor_SetAltController};
void (*AltControlTable[])(Motor *self, uint32_t timeout) = {NULL, RM_Motor_altControl, VESC_Motor_AltControl,
                                                            RMD_Motor_AltControl};
Motor *(*FindTable[])(CAN_Device device, uint8_t id)     = {NULL, RM_Motor_get, VESC_Motor_Find, RMD_Motor_Find};

void Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param) {
  if (self) {
    SetSpeedPIDTable[self->info.type](self, param);
  }
}
void Motor_SetAnglePID(Motor *self, PID_ControllerParam *param) {
  if (self) {
    SetAnglePIDTable[self->info.type](self, param);
  }
}
void Motor_Zero(Motor *self) {
  if (self) {
    ZeroTable[self->info.type](self);
  }
}
void Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout) {
  if (self) {
    SetCurrentTable[self->info.type](self, mA, timeout);
  }
}
void Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout) {
  if (self) {
    SetSpeedTable[self->info.type](self, rpm, timeout);
  }
}
void Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout) {
  if (self) {
    SetAngleTable[self->info.type](self, rad, timeout);
  }
}
void Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                            MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller, void *param)) {
  if (self) {
    SetAltControllerTable[self->info.type](self, alt_controller, param, alt_controller_update);
  }
}
void Motor_AltControl(Motor *self, uint32_t timeout) {
  if (self) {
    AltControlTable[self->info.type](self, timeout);
  }
}
Motor *CAN_Find_Motor(MotorType type, CAN_Device device, uint8_t id) { return FindTable[type](device, id); }
