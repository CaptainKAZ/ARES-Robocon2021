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
#include "encoder.h"
#include "sbus.h"
#include "feedback_task.h"
#include "user_lib.h"

Chassis chassis;

static uint8_t encoderUpdateId = 0;

static PID_ControllerParam chassisSwerveSteerSpeedPid = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = NORMAL_INT,
                                                         .kB           = 0,
                                                         .kP           = 5.0f,
                                                         .kI           = 1.5f,
                                                         .kD           = 0.1f,
                                                         .N            = 30,
                                                         .max_Iout     = 16000.0f};

static PID_ControllerParam chassisSwerveSteerAnglePid = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = NORMAL_INT,
                                                         .kB           = 0,
                                                         .kP           = 50000.0f,
                                                         .kI           = 0.0f,
                                                         .kD           = 1.0f,
                                                         .N            = 0.1f,
                                                         .max_Iout     = 16000.0f};

static PID_ControllerParam chassisSwerveDriveSpeedPid = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = NORMAL_INT,
                                                         .kB           = 0,
                                                         .kP           = 1.5f,
                                                         .kI           = 0.1f,
                                                         .kD           = 0.05f,
                                                         .N            = 50,
                                                         .max_Iout     = 500.0f};

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

  while (chassis.component.motor[0][0] == NULL || chassis.component.motor[0][1] == NULL ||
         chassis.component.motor[0][2] == NULL || chassis.component.motor[0][3] == NULL ||
         chassis.component.motor[1][0] == NULL || chassis.component.motor[1][1] == NULL ||
         chassis.component.motor[1][2] == NULL || chassis.component.motor[1][3] == NULL) {
    chassis.component.motor[0][0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 0);
    chassis.component.motor[0][1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 1);
    chassis.component.motor[0][2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 0);
    chassis.component.motor[0][3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
    chassis.component.motor[1][0] = CAN_Find_Motor(RMD_MOTOR, INTERNAL_CAN2, 0);
    chassis.component.motor[1][1] = CAN_Find_Motor(RMD_MOTOR, INTERNAL_CAN2, 1);
    chassis.component.motor[1][2] = CAN_Find_Motor(RMD_MOTOR, INTERNAL_CAN1, 0);
    chassis.component.motor[1][3] = CAN_Find_Motor(RMD_MOTOR, INTERNAL_CAN1, 1);
    vTaskDelay(10);
  }

  chassis.instruct.vx = chassis.instruct.vy = chassis.instruct.wz = 0;

  for (uint8_t i = 0; i < 4; i++) {
    Motor_SetSpeedPID(chassis.component.motor[0][i], &chassisSwerveSteerSpeedPid);
    Motor_SetSpeedPID(chassis.component.motor[1][i], &chassisSwerveDriveSpeedPid);
  }

  Swerve_init(&chassis.component.swerve[0], &canEncoder[0][0], chassis.component.motor[0][0], chassis.component.motor[1][0],
              SWERVE_ENCODER_RATIO, SWERVE_STEER_MOTOR_RATIO, SWERVE_DRIVE_MOTOR_RATIO, 0.1835f, -0.1685f,
              &chassisSwerveSteerAnglePid);
  Swerve_init(&chassis.component.swerve[1], &canEncoder[0][1], chassis.component.motor[0][1], chassis.component.motor[1][1],
              SWERVE_ENCODER_RATIO, SWERVE_STEER_MOTOR_RATIO, SWERVE_DRIVE_MOTOR_RATIO, 0.1835f, 0.1685f,
              &chassisSwerveSteerAnglePid);
  Swerve_init(&chassis.component.swerve[2], &canEncoder[1][2], chassis.component.motor[0][2], chassis.component.motor[1][2],
              SWERVE_ENCODER_RATIO, SWERVE_STEER_MOTOR_RATIO, SWERVE_DRIVE_MOTOR_RATIO, -0.1835f, 0.1685f,
              &chassisSwerveSteerAnglePid);
  Swerve_init(&chassis.component.swerve[3], &canEncoder[1][3], chassis.component.motor[0][3], chassis.component.motor[1][3],
              SWERVE_ENCODER_RATIO, SWERVE_STEER_MOTOR_RATIO, SWERVE_DRIVE_MOTOR_RATIO, -0.1835f, -0.1685f,
              &chassisSwerveSteerAnglePid);

  if (sbus.real.channel[SBUS_SD_CHANNEL] > 0.8f) {
    Swerve_zero(&chassis.component.swerve[0]);
    Swerve_zero(&chassis.component.swerve[1]);
    Swerve_zero(&chassis.component.swerve[2]);
    Swerve_zero(&chassis.component.swerve[3]);
  }

  feedback_register(&chassis.component.swerve->steerAngle, 4);
  feedback_register(&chassis.component.swerve->steerTarget, 5);
  feedback_register(&chassis.component.swerve[0].driveMotor->instruct.set, 6);
  feedback_register(&chassis.component.swerve[0].driveMotor->status.speed, 7);

  for (;;) {
    Chassis_getControl();
    if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
      Swerve_calcKinematics(&chassis.component.swerve[0], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calcKinematics(&chassis.component.swerve[1], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calcKinematics(&chassis.component.swerve[2], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);
      Swerve_calcKinematics(&chassis.component.swerve[3], chassis.instruct.vx, chassis.instruct.vy, chassis.instruct.wz);

      fp32 maxtarget = maxabs4f(chassis.component.swerve[0].driveTarget / chassis.component.swerve[0].driveMotorRatio,
                                chassis.component.swerve[1].driveTarget / chassis.component.swerve[1].driveMotorRatio,
                                chassis.component.swerve[2].driveTarget / chassis.component.swerve[2].driveMotorRatio,
                                chassis.component.swerve[3].driveTarget / chassis.component.swerve[3].driveMotorRatio);
      fp32 maxspeed  = maxabs4f(chassis.component.swerve[0].driveSpeed, chassis.component.swerve[1].driveSpeed,
                               chassis.component.swerve[2].driveSpeed, chassis.component.swerve[3].driveSpeed);

      if (maxtarget > 1010.0f) {
        chassis.component.swerve[0].driveTarget = chassis.component.swerve[0].driveTarget * 1010.0f / maxtarget;
        chassis.component.swerve[1].driveTarget = chassis.component.swerve[1].driveTarget * 1010.0f / maxtarget;
        chassis.component.swerve[2].driveTarget = chassis.component.swerve[2].driveTarget * 1010.0f / maxtarget;
        chassis.component.swerve[3].driveTarget = chassis.component.swerve[3].driveTarget * 1010.0f / maxtarget;
      }

      //        chassis.component.swerve[0].driveTarget = chassis.component.swerve[0].driveTarget * sbus.filtered.channel[SBUS_LY_CHANNEL];
      //        chassis.component.swerve[1].driveTarget = chassis.component.swerve[1].driveTarget * sbus.filtered.channel[SBUS_LY_CHANNEL];
      //        chassis.component.swerve[2].driveTarget = chassis.component.swerve[2].driveTarget * sbus.filtered.channel[SBUS_LY_CHANNEL];
      //        chassis.component.swerve[3].driveTarget = chassis.component.swerve[3].driveTarget * sbus.filtered.channel[SBUS_LY_CHANNEL];
      if (!(__fabsf(chassis.component.swerve[0].steerAngle - chassis.component.swerve[0].steerTarget) < DEG2RAD(10) &&
            __fabsf(chassis.component.swerve[1].steerAngle - chassis.component.swerve[1].steerTarget) < DEG2RAD(10) &&
            __fabsf(chassis.component.swerve[2].steerAngle - chassis.component.swerve[2].steerTarget) < DEG2RAD(10) &&
            __fabsf(chassis.component.swerve[3].steerAngle - chassis.component.swerve[3].steerTarget) < DEG2RAD(10))) {
        if (maxspeed < 0.5f) {
          chassis.component.swerve[0].driveTarget     = chassis.component.swerve[1].driveTarget =
              chassis.component.swerve[2].driveTarget = chassis.component.swerve[3].driveTarget = 0;
        }
      }

      Swerve_execute(&chassis.component.swerve[0], 2);
      Swerve_execute(&chassis.component.swerve[1], 2);
      Swerve_execute(&chassis.component.swerve[2], 2);
      Swerve_execute(&chassis.component.swerve[3], 2);
      Encoder_readValue(INTERNAL_CAN2, encoderUpdateId + 2);
      Encoder_readValue(INTERNAL_CAN1, encoderUpdateId + 2);
      encoderUpdateId++;
      encoderUpdateId = encoderUpdateId % 4;
    }
    vTaskDelay(1);
  }
}
