/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     quick_test_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    
  * @version  0.1
  * @date     2021-03-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quick_test_task.h"
#include "Motor.h"
#include "RM_motor.h"
#include "feedback_task.h"
#include "steer.h"
#include "encoder.h"
#include "PID.h"
#include "user_lib.h"
#include "quintic_trajectory.h"
#include "encoder.h"
#include "math.h"

static PID_ControllerParam testSteerParam = {.general.type = PIDPOS_CONTROLLER,
                                             .Int_type     = NORMAL_INT,
                                             .kB           = 0,
                                             .kP           = 900.0f,
                                             .kI           = 0.0f,
                                             .kD           = 3.0f,
                                             .N            = 0.1f,
                                             .max_Iout     = 10000.0f};

static PID_ControllerParam testMotor0SpeedParam = {.general.type = PIDPOS_CONTROLLER,
                                                   .Int_type     = NORMAL_INT,
                                                   .kB           = 0,
                                                   .kP           = 5.0f,
                                                   .kI           = 1.5f,
                                                   .kD           = 0.1f,
                                                   .N            = 30,
                                                   .max_Iout     = 16000.0f};

static PID_ControllerParam testMotor1SpeedParam = {.general.type = PIDPOS_CONTROLLER,
                                                   .Int_type     = NORMAL_INT,
                                                   .kB           = 0,
                                                   .kP           = 20.0f,
                                                   .kI           = 10.0f,
                                                   .kD           = 0.1f,
                                                   .N            = 30,
                                                   .max_Iout     = 16000.0f};

static Motor *  testMotor[2];
static Steer    testSteer;
static fp32     dir;
static fp32     vel;

void quick_test_task(void *argument) {
  vTaskDelay(200);

  while (testMotor[0] == NULL || testMotor[1] == NULL) {
    testMotor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 4);
    testMotor[1] = CAN_Find_Motor(RMD_MOTOR, INTERNAL_CAN1, 0);
    vTaskDelay(100);
  }

  Motor_SetSpeedPID(testMotor[0], &testMotor0SpeedParam);
  Motor_SetSpeedPID(testMotor[0], &testMotor1SpeedParam);

  Steer_init(&testSteer, &canEncoder[0][0], testMotor[0], STEER_ENCODER_RATIO, STEER_MOTOR_REDUCTION_RATIO, &testSteerParam);
  feedback_register(&testSteer.angle, FEEDBACK_STEER_ANGLE);
  feedback_register(&testSteer.target, FEEDBACK_STEER_TARGET);

  feedback_register(&testMotor[1]->instruct.set, FEEDBACK_DRIVE_TARGET);
  feedback_register(&testMotor[1]->status.speed, FEEDBACK_DRIVE_SPEED);

  for (;;) {
    if (sbus.channel[SBUS_RY_CHANNEL] != 0 || sbus.channel[SBUS_RX_CHANNEL] != 0) {
      dir = RAD2DEG(atan2f(sbus.channel[SBUS_RY_CHANNEL], sbus.channel[SBUS_RX_CHANNEL])) + 180.0f;
      vel = __sqrtf(sbus.channel[SBUS_RY_CHANNEL] * sbus.channel[SBUS_RY_CHANNEL] +
                    sbus.channel[SBUS_RX_CHANNEL] * sbus.channel[SBUS_RX_CHANNEL]);
      vel = vel < 1.0f ? vel : 1;
      if (dir - testSteer.angle > 90.0f) {
        dir -= 180.0f;
        vel = -vel;
      } else if (dir - testSteer.angle < -90.0f) {
        dir += 180.0f;
        vel = -vel;
      }
    } else {
      vel = 0;
    }
    vel *= 1000.0f;
    Steer_turn(&testSteer, dir, 20);
    Motor_SetSpeed(testMotor[1], vel, 20);
    vTaskDelay(1);
  }
}
