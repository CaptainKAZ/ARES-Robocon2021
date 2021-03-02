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

//电机控制的间隔，单位为ms
#define MOTOR_CTRL_TIME 1

typedef enum {
  NONE = 0,
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
  INSTRUCT_CURRENT = 0, //电流控制，单位为mA
  INSTRUCT_SPEED,       //转速控制，单位为RPM
  INSTRUCT_ANGLE,       //角度控制，弧度制
  INSTRUCT_ALTERNATIVE, //使用备选控制器
} MotorInstructType;

typedef struct {
  MotorInstructType type;
  fp32              set;
  uint32_t          timeout; //控制超时，单位为ms
} MotorInstruct;

typedef struct {
  fp32     current;
  fp32     speed;
  fp32     angle;
  fp32     temperature;
} MotorStatus;

typedef struct Motor {
  MotorInfo     info;
  MotorStatus   status;
  MotorInstruct instruct;
} Motor;

#endif
