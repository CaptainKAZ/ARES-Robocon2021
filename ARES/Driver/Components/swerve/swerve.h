/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     swerve.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    舵轮控制算法
  * @version  0.1
  * @date     2021-05-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef SWERVE_H
#define SWERVE_H

#include "main.h"
#include "can_encoder.h"
#include "rm_motor.h"

#define SWERVE_EXTERNAL_ENCODER 0

/*
坐标系定义
        X
        ↑
        |
Y ←---- ✛ -----
        |
        |
同时逆时针为旋转正
*/

#define SWERVE_STEER_MOTOR_RATIO (8.125f)
#define SWERVE_WHEEL_RADIUS (0.05f)
#define SWERVE_DRIVE_MOTOR_RATIO (3.0f)
#define SWERVE_SPEED_RATIO (30.0f * SWERVE_DRIVE_MOTOR_RATIO / SWERVE_WHEEL_RADIUS / PI)

typedef struct {

  fp32 *encoder;

  Motor *steerMotor;
  Motor *driveMotor;

  fp32 speedRatio; //从期望线速度到驱动电机转速的比例
  fp32 encoderCenter;

  fp32 speedTarget; //航向目标(m/s)
  fp32 angleTarget; //舵向目标(rad)

  fp32 rho;   //相对与中心的距离(m)
  fp32 theta; //相对于中心的角度(RAD)

  uint8_t  zeroed;
} Swerve;

extern uint8_t Swerve_init(Swerve *swerve, fp32 *encoder, Motor *steerMotor, Motor *driveMotor, fp32 steerMotorRatio,
                           fp32 speedRatio, fp32 x, fp32 y, fp32 encoderCenter);
extern void    Swerve_calc(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz);
extern void    Swerve_execute(Swerve *swerve, uint32_t timeout);
extern uint8_t Swerve_goZero(Swerve *swerve);


#endif
