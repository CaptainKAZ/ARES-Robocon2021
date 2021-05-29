/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     steer.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    减速舵轮
  * @version  0.1
  * @date     2021-04-18
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef STEER_H
#define STEER_H
#include "main.h"
#include "motor.h"
#include "PID.h"
#include "encoder.h"

#define STEER_ENCODER_DIRECTION (-1)
#define STEER_ENCODER_REDUCTION_RATIO (4)
#define STEER_ENCODER_RESOLUTION (1024)
#define STEER_ENCODER_RATIO (STEER_ENCODER_DIRECTION * STEER_ENCODER_REDUCTION_RATIO * STEER_ENCODER_RESOLUTION)
#define STEER_MOTOR_REDUCTION_RATIO (4.0f * M2006_REDUCTION_RATIO)

typedef struct {
  CanEncoder *   encoder;
  Motor *        motor;
  int32_t        encoderRatio;
  int32_t        motorRatio;
  fp32           angle;  //角度 (DEG)
  fp32           target; //角度 (DEG)
  PID_Controller controller;
} Steer;

extern void Steer_init(Steer *steer, CanEncoder *encoder, Motor *motor, int32_t encRatio, fp32 motorReductionRatio,
                       PID_ControllerParam *pidParam);
extern void Steer_turn(Steer *steer, fp32 deg, uint32_t timeout);

#endif
