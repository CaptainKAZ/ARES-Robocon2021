/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     chassis_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    底盘控制任务
  * @version  0.1
  * @date     2021-04-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "motor.h"
#include "rm_motor.h"
#include "swerve.h"
#include "pwm_encoder.h"
#include "sbus.h"
#include "feedback_task.h"
#include "user_lib.h"

Chassis chassis;

fp32 rxTime=0;

static PID_ControllerParam chassisSwerveSteerSpeedPid = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = NORMAL_INT,
                                                         .kB           = 0,
                                                         .kP           = 16.0f,
                                                         .kI           = 1.0f,
                                                         .kD           = 0.0f,
                                                         .N            = 30,
                                                         .max_Iout     = 16000.0f};

static PID_ControllerParam chassisSwerveSteerAnglePid = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = NORMAL_INT,
                                                         .kB           = 0,
                                                         .kP           = 120.0f,
                                                         .kI           = 0.0f,
                                                         .kD           = 5.0f,
                                                         .N            = 0.0f,
                                                         .max_Iout     = 16000.0f};



static void Chassis_getControl() {
  Sbus_lpf();
  if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
    chassis.instruct.vx = CHASSIS_MAX_TRANS_SPEED * sbus.filtered.channel[SBUS_RY_CHANNEL];
    chassis.instruct.vy = -CHASSIS_MAX_TRANS_SPEED * sbus.filtered.channel[SBUS_RX_CHANNEL];
    chassis.instruct.wz = -CHASSIS_MAX_ROT_SPEED * sbus.filtered.channel[SBUS_LX_CHANNEL];
  } else {
    chassis.instruct.vx = chassis.instruct.vy = chassis.instruct.wz = 0;
  }
}

void chassis_task(void *args) {
  vTaskDelay(623);
  while (sbus.real.updateTime == 0) {
    vTaskDelay(10);
  }
  while (sbus.real.channel[SBUS_SA_CHANNEL]<0.8f){
     vTaskDelay(10);
  }
  while (chassis.component.motor[0][0] == NULL || chassis.component.motor[0][1] == NULL ||
         chassis.component.motor[0][2] == NULL || chassis.component.motor[0][3] == NULL ||
         chassis.component.motor[1][0] == NULL || chassis.component.motor[1][1] == NULL ||
         chassis.component.motor[1][2] == NULL || chassis.component.motor[1][3] == NULL) {
    chassis.component.motor[0][0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 3);
    chassis.component.motor[0][1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 2);
    chassis.component.motor[0][2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 1);
    chassis.component.motor[0][3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
    chassis.component.motor[1][0] = CAN_Find_Motor(VESC_MOTOR, INTERNAL_CAN2, 4);
    chassis.component.motor[1][1] = CAN_Find_Motor(VESC_MOTOR, INTERNAL_CAN1, 1);
    chassis.component.motor[1][2] = CAN_Find_Motor(VESC_MOTOR, INTERNAL_CAN2, 3);
    chassis.component.motor[1][3] = CAN_Find_Motor(VESC_MOTOR, INTERNAL_CAN1, 2);
    vTaskDelay(10);
  }

  feedback_register(&((RM_Motor*)chassis.component.motor[0][1])->general.status.angleOutput, 0);
  feedback_register(&((RM_Motor *)chassis.component.motor[0][1])->general.status.speedOutput, 1);
  rxTime = xTaskGetTickCount() - ((RM_Motor *)chassis.component.motor[0][1])->rx_timestamp;
  feedback_register(&rxTime, 2);
  feedback_register(&((RM_Motor *)chassis.component.motor[0][1])->angle_pid.out[0], 3);

  chassis.instruct.vx = chassis.instruct.vy = chassis.instruct.wz = 0;

  for (uint8_t i = 0; i < 4; i++) {
    Motor_SetSpeedPID(chassis.component.motor[0][i], &chassisSwerveSteerSpeedPid);
    Motor_SetAnglePID(chassis.component.motor[0][i], &chassisSwerveSteerAnglePid);
  }

  Swerve_init(&chassis.component.swerve[0], &pwmEncoder[0], chassis.component.motor[0][0], chassis.component.motor[1][0],
              SWERVE_STEER_MOTOR_RATIO, SWERVE_SPEED_RATIO, 0.36f, 0.36f, 0);
  Swerve_init(&chassis.component.swerve[1], &pwmEncoder[1], chassis.component.motor[0][1], chassis.component.motor[1][1],
              SWERVE_STEER_MOTOR_RATIO, SWERVE_SPEED_RATIO, 0.36f, -0.36f, 0);
  Swerve_init(&chassis.component.swerve[2], &pwmEncoder[2], chassis.component.motor[0][2], chassis.component.motor[1][2],
              SWERVE_STEER_MOTOR_RATIO, SWERVE_SPEED_RATIO, -0.36f, 0.36f, 0);
  Swerve_init(&chassis.component.swerve[3], &pwmEncoder[3], chassis.component.motor[0][3], chassis.component.motor[1][3],
              SWERVE_STEER_MOTOR_RATIO, SWERVE_SPEED_RATIO, -0.36f, -0.36f, 0);

  //  while (1) {
  //    if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
  //      if (Swerve_goZero(&chassis.component.swerve[0]) & Swerve_goZero(&chassis.component.swerve[0]) &
  //          Swerve_goZero(&chassis.component.swerve[0]) & Swerve_goZero(&chassis.component.swerve[0])) {
  //        break;
  //      }
  //    }
  //    vTaskDelay(1) ;
  //  }

  for (;;) {
    Chassis_getControl();
    rxTime = xTaskGetTickCount() - ((RM_Motor *)chassis.component.motor[0][1])->rx_timestamp;
    if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
      Swerve_calc(&chassis.component.swerve[0], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calc(&chassis.component.swerve[1], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calc(&chassis.component.swerve[2], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calc(&chassis.component.swerve[3], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_execute(&chassis.component.swerve[0], 2);
      Swerve_execute(&chassis.component.swerve[1], 2);
      Swerve_execute(&chassis.component.swerve[2], 2);
      Swerve_execute(&chassis.component.swerve[3], 2);
    }
    vTaskDelay(1);
  }
}
