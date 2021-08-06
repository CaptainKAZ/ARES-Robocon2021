/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     fetch.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    取箭
  * @version  0.1
  * @date     2021-07-11
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quick_test_task.h"
#include "bezier_trajectory.h"
#include "PID.h"
BezierTraj traj[2];
// fp32       points[2][6] = {{DEG2RAD(-5), DEG2RAD(-40), DEG2RAD(-40), DEG2RAD(-90.0), DEG2RAD(-90.0), DEG2RAD(-110)},
//                     {DEG2RAD(-26), DEG2RAD(-0), DEG2RAD(-5), DEG2RAD(-180.5), DEG2RAD(-180.5), DEG2RAD(-144.5)}};
fp32       points[2][6] = {{DEG2RAD(-5), DEG2RAD(-40), DEG2RAD(-40), DEG2RAD(-90.0), DEG2RAD(-90.0), DEG2RAD(-110)},
                    {DEG2RAD(-33), DEG2RAD(-0), DEG2RAD(-5), DEG2RAD(-180.5), DEG2RAD(-180.5), DEG2RAD(-144.5)}};
Motor *    testMotor[2];
fp32       totalTime = 1000;
uint8_t    zeroed[2] = {0};
uint32_t   motorStuckTime[2];
int32_t    timebase;
fp32       trigger;

static PID_ControllerParam testSpeedParamLow = {.general.type = PIDPOS_CONTROLLER,
                                                .Int_type     = BACK_CALCULATION_INT,
                                                .kB           = 0,
                                                .kP           = 20.0f,
                                                .kI           = 2.0f,
                                                .kD           = 0.0f,
                                                .N            = 0.0f,
                                                .max_Iout     = 10000.0f};

static PID_ControllerParam testSpeedParamHigh = {.general.type = PIDPOS_CONTROLLER,
                                                 .Int_type     = BACK_CALCULATION_INT,
                                                 .kB           = 0,
                                                 .kP           = 12.0f,
                                                 .kI           = 2.0f,
                                                 .kD           = 0.5f,
                                                 .N            = 0.0f,
                                                 .max_Iout     = 10000.0f};
static PID_ControllerParam testAngleParamLow = {.general.type = PIDPOS_CONTROLLER,
                                                .Int_type     = BACK_CALCULATION_INT,
                                                .kB           = 0,
                                                .kP           = 20.0f,
                                                .kI           = 2.0f,
                                                .kD           = 0.0f,
                                                .N            = 0.0f,
                                                .max_Iout     = 10000.0f};

static PID_ControllerParam testAngleParamHigh = {.general.type = PIDPOS_CONTROLLER,
                                                 .Int_type     = BACK_CALCULATION_INT,
                                                 .kB           = 0,
                                                 .kP           = 12.0f,
                                                 .kI           = 2.0f,
                                                 .kD           = 0.5f,
                                                 .N            = 0.0f,
                                                 .max_Iout     = 10000.0f};

fp32 setAngle[2];

void quick_test_task() {
  vTaskDelay(1024);
  for (;;) {
    testMotor[0] = CAN_Find_Motor(RM_MOTOR, EXTERNAL_CAN1, 0);
    testMotor[1] = CAN_Find_Motor(RM_MOTOR, EXTERNAL_CAN1, 1);
    for (uint8_t i = 0; i < 2; i++) {
      if (testMotor[i] == NULL) {
        vTaskDelay(20);
        continue;
      }
    }
    break;
  }
  RM_Motor_setAsM3508(testMotor[0]);
  testMotor[0]->reductionRatio *= 2;
  RM_Motor_setAsM3508(testMotor[1]);
  bezierTraj_init(&traj[0], points[0], totalTime);
  bezierTraj_init(&traj[1], points[1], totalTime);
  Motor_SetSpeedPID(testMotor[0], &testSpeedParamLow);
  Motor_SetSpeedPID(testMotor[1], &testSpeedParamHigh);
  Motor_SetAnglePID(testMotor[0], &testAngleParamLow);
  Motor_SetAnglePID(testMotor[1], &testAngleParamHigh);
  for (;;) {
    if (!zeroed[0]) {
      Motor_SetSpeed(testMotor[0], 100, 2);
      if (testMotor[0]->status.speedOutput <= 10) {
        motorStuckTime[0]++;
      } else {
        motorStuckTime[0] = 0;
      }
      if (motorStuckTime[0] > 1000) {
        Motor_Zero(testMotor[0]);
        zeroed[0] = 1;
      }
    } else if (!zeroed[1]) {
      Motor_SetSpeed(testMotor[1], 100, 2);
      if (testMotor[1]->status.speedOutput <= 10) {
        motorStuckTime[1]++;
      } else {
        motorStuckTime[1] = 0;
      }
      if (motorStuckTime[1] > 1000) {
        Motor_Zero(testMotor[1]);
        zeroed[1] = 1;
        timebase=-1;
      }
    }
    if (zeroed[0] && zeroed[1]) {
      if (sbus.real.channel[9] > 0.8f && timebase <= 0) {
        timebase = 3000;
        bezierTraj_init(&traj[0], points[0], totalTime);
        bezierTraj_init(&traj[1], points[1], totalTime);
        bezierTraj_start(&traj[0]);
        bezierTraj_start(&traj[1]);
      } else if (timebase > 0) {
        timebase--;
        bezierTraj_calc(&traj[0], TRAJ_DIR_FORWARD, &setAngle[0], NULL, NULL);
        bezierTraj_calc(&traj[1], TRAJ_DIR_FORWARD, &setAngle[1], NULL, NULL);
      } else if (timebase == 0) {
        bezierTraj_start(&traj[0]);
        bezierTraj_start(&traj[1]);
        timebase--;
        bezierTraj_calc(&traj[1], TRAJ_DIR_BACKWARD, &setAngle[1], NULL, NULL);
        bezierTraj_calc(&traj[0], TRAJ_DIR_BACKWARD, &setAngle[0], NULL, NULL);
      } else if (timebase < 0) {
        bezierTraj_calc(&traj[1], TRAJ_DIR_BACKWARD, &setAngle[1], NULL, NULL);           
        bezierTraj_calc(&traj[0], TRAJ_DIR_BACKWARD, &setAngle[0], NULL, NULL);
      }
      Motor_SetAngle(testMotor[0], setAngle[0], 2);
      Motor_SetAngle(testMotor[1], setAngle[1], 2);
    }
    vTaskDelay(1);
  }
}
