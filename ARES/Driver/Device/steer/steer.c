/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     steer.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    减速舵轮
  * @version  0.1
  * @date     2021-04-18
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "steer.h"
#include "rm_motor.h"
#include "cmsis_os.h"
#include "user_lib.h"

static fp32                steerConsterainOHLim     = 20000.0f;
static fp32                steerConsterainOLLim     = -20000.0f;
static fp32                steerConsterainILoopLLim = 0;
static fp32                steerConsterainILoopHLim = 360.0;
static ControllerConstrain steerConsterain          = {.I_loop_Hlim = &steerConsterainILoopHLim,
                                              .I_loop_Llim = &steerConsterainILoopLLim,
                                              .O_Hlim      = &steerConsterainOHLim,
                                              .O_Llim      = &steerConsterainOLLim};

static MotorInstructType Steer_controllerUpdate(Motor *motor, Controller *controller, void *param) {
#define STEER ((Steer *)param)
  
    if (STEER->encoderRatio > 0) {
      STEER->angle = (fp32)(((STEER->encoder->value)) % (STEER->encoderRatio)) / (fp32)(STEER->encoderRatio) * 360.0f;
    } else {
      STEER->angle = (360.0f - (fp32)(((STEER->encoder->value)) % (-STEER->encoderRatio)) / (fp32)(-STEER->encoderRatio) * 360.0f);
    }

//    STEER->motor->status.cumulative_turn = (int32_t)(STEER->angle * STEER->motorRatio) / 360;
//    STEER->motor->status.zero            = DEG2RAD((int32_t)(STEER->angle * STEER->motorRatio) % 360);
  
  fp32 rpmset = controllerUpdate(controller, &STEER->target, &STEER->angle, NULL);
  ((RM_Motor *)motor)->set_current = controllerUpdate((Controller *)&(((RM_Motor *)motor)->speed_pid), &rpmset, &motor->status.speed, NULL);

  return INSTRUCT_CURRENT;

#undef STEER
}

void Steer_init(Steer *steer, CanEncoder *encoder, Motor *motor, int32_t encRatio, fp32 motorReductionRatio,
                PID_ControllerParam *pidParam) {
  steer->motor        = motor;
  steer->encoder      = encoder;
  steer->motorRatio   = motorReductionRatio;
  steer->encoderRatio = encRatio;
  while (encoder->value == 0) {
    vTaskDelay(100);
  }
  if (steer->encoderRatio > 0) {
    steer->angle = (fp32)(((steer->encoder->value)) % (steer->encoderRatio)) / (fp32)(steer->encoderRatio) * 360.0f;
  } else {
    steer->angle = 360.0f - (fp32)(((steer->encoder->value)) % (-steer->encoderRatio)) / (fp32)(-steer->encoderRatio) * 360.0f;
  }
  steer->target                        = 0;
//  steer->motor->status.cumulative_turn = (int32_t)(steer->angle * steer->motorRatio) / 360;
//  steer->motor->status.zero            = DEG2RAD((int32_t)(steer->angle * steer->motorRatio) % 360);
  PID_ControllerInit(&steer->controller, &steerConsterain, pidParam, 0.002f);
  Motor_SetAltController(steer->motor, (Controller *)&steer->controller, (void *)steer, Steer_controllerUpdate);
}

void Steer_turn(Steer *steer, fp32 deg, uint32_t timeout) {
  steer->target = deg;
  Motor_AltControl(steer->motor, timeout);
}
