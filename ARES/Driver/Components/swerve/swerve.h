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
#include "encoder.h"
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

#define SWERVE_ENCODER_DIRECTION (-1)
#define SWERVE_ENCODER_REDUCTION_RATIO (4)
#define SWERVE_ENCODER_RESOLUTION (1024)
#define SWERVE_ENCODER_RATIO (SWERVE_ENCODER_DIRECTION * SWERVE_ENCODER_REDUCTION_RATIO * SWERVE_ENCODER_RESOLUTION)
#define SWERVE_STEER_MOTOR_RATIO (4.0f * M2006_REDUCTION_RATIO)
#define SWERVE_WHEEL_RADIUS (0.05f)
#define SWERVE_DRIVE_MOTOR_RATIO (SWERVE_WHEEL_RADIUS * PI / 30.0f)

typedef struct {

  CanEncoder *encoder;

  Motor *steerMotor;
  Motor *driveMotor;

  int32_t encoderRatio;
  fp32    steerMotorRatio; //转向电机减速比
  fp32    driveMotorRatio; //驱动电机到m/s的比例

  fp32 steerAngle;  //转向角度 (RAD)
  fp32 steerTarget; //设定角度 (RAD)
  fp32 driveSpeed;  //航向速度(m/s)
  fp32 driveTarget; //航向目标(m/s)

  fp32 rho;   //相对与中心的距离(m)
  fp32 theta; //相对于中心的角度(RAD)

  uint32_t updateTime;
} Swerve;

extern uint8_t Swerve_init(Swerve *swerve, CanEncoder *encoder, Motor *steerMotor, Motor *driveMotor, int32_t encoderRatio,
                           fp32 steerMotorRatio, fp32 driveMotorRatio, fp32 x, fp32 y, PID_ControllerParam *steerPidParam);
extern void    Swerve_update(Swerve *swerve);
extern void    Swerve_calcKinematics(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz);
extern void    Swerve_setTargets(Swerve *swerve, fp32 angle, fp32 speed);
extern void    Swerve_execute(Swerve *swerve, int32_t timeout);
extern void    Swerve_zero(Swerve *swerve);

#endif
