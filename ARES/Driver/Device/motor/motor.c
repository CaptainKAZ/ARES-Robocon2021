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
#include "VESC_motor.h"
#include "RM_motor.h"
#include "RMD_motor.h"

extern void RM_Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param);
extern void RMD_Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param);

extern void RM_Motor_SetAnglePID(Motor *self, PID_ControllerParam *param);
extern void RMD_Motor_SetAnglePID(Motor *self, PID_ControllerParam *param);

extern void RMD_Motor_Zero(Motor *self);
extern void RM_Motor_Zero(Motor *self);
extern void VESC_Motor_Zero(Motor *self);

extern void RM_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);
extern void RMD_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);
extern void VESC_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);

extern void RM_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);
extern void VESC_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);
extern void RMD_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);

extern void RM_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);
extern void RMD_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);
extern void VESC_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);

extern void RM_Motor_SetAltController(Motor *self, Controller *alt_controller,
                                      fp32 (*alt_controller_update)(Motor *motor, Controller *controller));
extern void RMD_Motor_SetAltController(Motor *self, Controller *alt_controller,
                                       fp32 (*alt_controller_update)(Motor *motor, Controller *controller));
extern void VESC_Motor_SetAltController(Motor *self, Controller *alt_controller,
                                        fp32 (*alt_controller_update)(Motor *motor, Controller *controller));

extern void RM_Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout);
extern void RMD_Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout);
extern void VESC_Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout);

extern Motor *RM_Motor_Find(CAN_Device device, uint8_t id);
extern Motor *RMD_Motor_Find(CAN_Device device, uint8_t id);
extern Motor *VESC_Motor_Find(CAN_Device device, uint8_t id);

void (*SetSpeedPIDTable[])(Motor *self, PID_ControllerParam *param)    = {NULL, RM_Motor_SetSpeedPID, NULL,
                                                                       RMD_Motor_SetSpeedPID};
void (*SetAnglePIDTable[])(Motor *self, PID_ControllerParam *param) = {NULL, RM_Motor_SetAnglePID, NULL,
                                                                       RMD_Motor_SetAnglePID};
void (*ZeroTable[])(Motor *self)                                    = {NULL, RM_Motor_Zero, VESC_Motor_Zero, RMD_Motor_Zero};
void (*SetCurrentTable[])(Motor *self, fp32 current, uint32_t timeout) = {NULL, RM_Motor_SetCurrent, VESC_Motor_SetCurrent,
                                                                          RMD_Motor_SetCurrent};
void (*SetSpeedTable[])(Motor *self, fp32 rpm, uint32_t timeout)       = {NULL, RM_Motor_SetSpeed, VESC_Motor_SetSpeed,
                                                                    RMD_Motor_SetSpeed};
void (*SetAngleTable[])(Motor *self, fp32 rad, uint32_t timeout)       = {NULL, RM_Motor_SetAngle, VESC_Motor_SetAngle,
                                                                    RMD_Motor_SetAngle};
void (*SetAltControllerTable[])(Motor *self, Controller *alt_controller,
                                fp32 (*alt_controller_update)(Motor *motor, Controller *controller)) = {
    NULL, RM_Motor_SetAltController, VESC_Motor_SetAltController, RMD_Motor_SetAltController};
void (*AltControlTable[])(Motor *self, MotorInstructType type,
                          uint32_t timeout) = {NULL, RM_Motor_AltControl, VESC_Motor_AltControl, RMD_Motor_AltControl};
Motor *(*FindTable[])(CAN_Device device, uint8_t id) = {NULL, RM_Motor_Find, VESC_Motor_Find, RMD_Motor_Find};

void Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param) { SetSpeedPIDTable[self->info.type](self, param); }
void Motor_SetAnglePID(Motor *self, PID_ControllerParam *param) { SetAnglePIDTable[self->info.type](self, param); }
void Motor_Zero(Motor *self) { ZeroTable[self->info.type](self); }
void Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout) { SetCurrentTable[self->info.type](self, mA, timeout); }
void Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout) { SetSpeedTable[self->info.type](self, rpm, timeout); }
void Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout) { SetAngleTable[self->info.type](self, rad, timeout); }
void Motor_SetAltController(Motor *self, Controller *alt_controller,
                            fp32 (*alt_controller_update)(Motor *motor, Controller *controller)) {
  SetAltControllerTable[self->info.type](self, alt_controller, alt_controller_update);
}
void Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout) {
  AltControlTable[self->info.type](self, type, timeout);
}
Motor *CAN_Find_Motor(MotorType type, CAN_Device device, uint8_t id) {return FindTable[type](device, id); }
