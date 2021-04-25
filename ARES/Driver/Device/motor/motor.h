/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     motor.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    电机类驱动程序
  * @version  0.1
  * @date     2021-02-21
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "can_comm.h"
#include "PID.h"

//电机控制的间隔，单位为ms
#define MOTOR_CTRL_TIME 1

typedef enum {
  NONE_MOTOR = 0,
  RM_MOTOR,
  VESC_MOTOR,
  RMD_MOTOR,
} MotorType;

typedef struct {
  MotorType  type;
  CAN_Device device;
  uint8_t    id;
} MotorInfo;

typedef enum {
  INSTRUCT_EASE = 0,    //不控制 不发出CAN帧 节约带宽
  INSTRUCT_CURRENT,     //电流控制，单位为mA
  INSTRUCT_SPEED,       //转速控制，单位为RPM
  INSTRUCT_ANGLE,       //角度控制，弧度制
  INSTRUCT_ALTERNATIVE, //使用备选控制器
} MotorInstructType;

typedef struct {
  MotorInstructType type;
  fp32              set;
  int32_t           timeout; //控制超时，单位为ms
} MotorInstruct;

typedef struct {
  fp32    current;
  fp32    speed;
  fp32    angle; //弧度制
  fp32    temperature;
  fp32    zero;
  int32_t cumulative_turn;
} MotorStatus;

typedef struct Motor {
  MotorInfo     info;
  MotorStatus   status;
  MotorInstruct instruct;
  Controller *  alt_controller;
  void *        alt_controller_param;
  MotorInstructType (*alt_controller_update)(struct Motor *motor, Controller *controller, void *param);
} Motor;

extern void Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param);
extern void Motor_SetAnglePID(Motor *self, PID_ControllerParam *param);
extern void Motor_Zero(Motor *self);
extern void Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout);
extern void Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout);
extern void Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout);
extern void Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                                   MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                              void *param));
extern void Motor_AltControl(Motor *self, uint32_t timeout);
Motor *     CAN_Find_Motor(MotorType type, CAN_Device device, uint8_t id);

#endif
