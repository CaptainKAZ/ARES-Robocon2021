/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     dr_chassis.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    DR底盘控制程序
  * @version  0.1
  * @date     2021-07-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "dr_chassis.h"
#include "motor.h"
#include "sbus.h"
#include "fsm.h"
#include "cmsis_os.h"
#include "rm_motor.h"
#include "ins_task.h"
#include "arm_math.h"
#include "ops.h"
#include "sbus.h"

DrChassis drChassis;

fp32 expandFlag = -1;
fp32 chassisLockX;
fp32 chassisLockY;
fp32 chassisLockYaw;

PID_ControllerParam chassisMotorSpeedPidParam = {.general  = PIDPOS_CONTROLLER,
                                                 .Int_type = BACK_CALCULATION_INT,
                                                 .kP       = 14.0f,
                                                 .kI       = 20.0f,
                                                 .kD       = 0.1f,
                                                 .kB       = 0.2f,
                                                 .max_Iout = 16384,
                                                 .N        = 100};

PID_ControllerParam chassisMotorAnglePidParam = {.general  = PIDPOS_CONTROLLER,
                                                 .Int_type = BACK_CALCULATION_INT,
                                                 .kP       = 1.0f,
                                                 .kI       = 0.0f,
                                                 .kD       = 0.0f,
                                                 .kB       = 0.0f,
                                                 .max_Iout = 20000,
                                                 .N        = 20};

PID_Controller             chassisXPid;
PID_ControllerParam        chassisXPidParam  = {.general  = PIDPOS_CONTROLLER,
                                        .Int_type = BACK_CALCULATION_INT,
                                        .kP       = 1.0f,
                                        .kI       = 0.0f,
                                        .kD       = 0.0f,
                                        .kB       = 0.3f,
                                        .max_Iout = 5000,
                                        .N        = 70};
static fp32                chassisXOHlim     = 5;
static fp32                chassisXOLlim     = -5;
static fp32                chassisXILoopHlim = 0;
static fp32                chassisXILoopLlim = 0;
static ControllerConstrain chassisXConstrain = {
    .I_loop_Hlim = &chassisXILoopHlim, .I_loop_Llim = &chassisXILoopLlim, .O_Hlim = &chassisXOHlim, .O_Llim = &chassisXOLlim};

PID_Controller             chassisYPid;
PID_ControllerParam        chassisYPidParam  = {.general  = PIDPOS_CONTROLLER,
                                        .Int_type = BACK_CALCULATION_INT,
                                        .kP       = 1.0f,
                                        .kI       = 0.0f,
                                        .kD       = 0.0f,
                                        .kB       = 0.3f,
                                        .max_Iout = 5000,
                                        .N        = 70};
static fp32                chassisYOHlim     = 5;
static fp32                chassisYOLlim     = -5;
static fp32                chassisYILoopHlim = 0;
static fp32                chassisYILoopLlim = 0;
static ControllerConstrain chassisYConstrain = {
    .I_loop_Hlim = &chassisYILoopHlim, .I_loop_Llim = &chassisYILoopLlim, .O_Hlim = &chassisYOHlim, .O_Llim = &chassisYOLlim};

PID_Controller             chassisYawPid;
PID_ControllerParam        chassisYawPidParam  = {.general  = PIDPOS_CONTROLLER,
                                          .Int_type = BACK_CALCULATION_INT,
                                          .kP       = 1.0f,
                                          .kI       = 0.0f,
                                          .kD       = 0.0f,
                                          .kB       = 0.3f,
                                          .max_Iout = 5000,
                                          .N        = 70};
static fp32                chassisYawOHlim     = 5;
static fp32                chassisYawOLlim     = -5;
static fp32                chassisYawILoopHlim = 0;
static fp32                chassisYawILoopLlim = 0;
static ControllerConstrain chassisYawConstrain = {.I_loop_Hlim = &chassisYawILoopHlim,
                                                  .I_loop_Llim = &chassisYawILoopLlim,
                                                  .O_Hlim      = &chassisYawOHlim,
                                                  .O_Llim      = &chassisYawOLlim};

static void   chassisEaseSwitchIn(StateMachine *fsm, void *param);
static State *chassisEaseTrans(StateMachine *fsm, void *param);
static void   chassisEaseExe(StateMachine *fsm, void *param);
State         chassisEase = {.param            = NULL,
                     .onSwitchIn       = chassisEaseSwitchIn,
                     .onSwitchOut      = NULL,
                     .stateExecute     = chassisEaseExe,
                     .stateTransistion = chassisEaseTrans};

static void   chassisLockMoveSwitchIn(StateMachine *fsm, void *param);
static State *chassisLockMoveTrans(StateMachine *fsm, void *param);
static void   chassisLockMoveExe(StateMachine *fsm, void *param);
State         chassisLockMove = {.param            = NULL,
                         .onSwitchIn       = chassisLockMoveSwitchIn,
                         .onSwitchOut      = NULL,
                         .stateExecute     = chassisLockMoveExe,
                         .stateTransistion = chassisLockMoveTrans};

static void   chassisMoveSwitchIn(StateMachine *fsm, void *param);
static State *chassisMoveTrans(StateMachine *fsm, void *param);
static void   chassisMoveExe(StateMachine *fsm, void *param);
State         chassisMove = {.param            = NULL,
                     .onSwitchIn       = chassisMoveSwitchIn,
                     .onSwitchOut      = NULL,
                     .stateExecute     = chassisMoveExe,
                     .stateTransistion = chassisMoveTrans};

static void chassisLockExe(StateMachine *fsm, void *param);
State       chassisLock = {.param            = NULL,
                     .onSwitchIn       = chassisLockMoveSwitchIn,
                     .onSwitchOut      = NULL,
                     .stateExecute     = chassisLockExe,
                     .stateTransistion = chassisLockMoveTrans};



/**
  * @brief    底盘反馈：可以进一步升级为底盘观测器
  * 
  * @param    chassis   底盘结构体
  */
void chassisFeedback(DrChassis *chassis) {
  if (xTaskGetTickCount() != chassis->feedback.updateTime) {
    for (uint8_t i = 0; i < 4; i++) {
      chassis->feedback.wheelSpeed[i] =
          (chassis->motor[i]->status.speedOutput - chassis->motor[i + 4]->status.speedOutput) / 60 * PI * WHEEL_RADIUS;
    }
#ifndef CHASSIS_FEEDBACK_A_IMU
    // 使用底盘电流作为加速度反馈
    chassis->feedback.a.x = (-SQRT2 * chassis->motor[0]->status.current + SQRT2 * chassis->motor[1]->status.current +
                             SQRT2 * chassis->motor[2]->status.current - SQRT2 * chassis->motor[3]->status.current) *
                            0.25f;
    chassis->feedback.a.y = (-SQRT2 * chassis->motor[0]->status.current - SQRT2 * chassis->motor[1]->status.current +
                             SQRT2 * chassis->motor[2]->status.current + SQRT2 * chassis->motor[3]->status.current) *
                            0.25f;
    chassis->feedback.a.z = (-SQRT2 * chassis->motor[0]->status.current - SQRT2 * chassis->motor[1]->status.current -
                             SQRT2 * chassis->motor[2]->status.current - SQRT2 * chassis->motor[3]->status.current) *
                            0.25f * chassis->wheelCenterDistance;
#else
    // 使用加速度计作为位置反馈
    chassis->feedback.a.x = motionFX_output.linear_acceleration_9X[0];
    chassis->feedback.a.y = motionFX_output.linear_acceleration_9X[1];
    chassis->feedback.a.z = motionFX_output.rotation_9X[2];
    //TODO:滤波融合
#endif
    //加速度计没法反馈展开加速度，只能使用电流
    chassis->feedback.a.expand = (SQRT2 * chassis->motor[0]->status.current - SQRT2 * chassis->motor[1]->status.current +
                                  SQRT2 * chassis->motor[2]->status.current - SQRT2 * chassis->motor[3]->status.current) *
                                 0.25f;
#ifndef CHASSIS_FEEDBACK_V_OPS
    chassis->feedback.v.x = (-SQRT2 * chassis->feedback.wheelSpeed[0] + SQRT2 * chassis->feedback.wheelSpeed[1] +
                             SQRT2 * chassis->feedback.wheelSpeed[2] - SQRT2 * chassis->feedback.wheelSpeed[3]) *
                            0.25f;
    chassis->feedback.v.y = (-SQRT2 * chassis->feedback.wheelSpeed[0] - SQRT2 * chassis->feedback.wheelSpeed[1] +
                             SQRT2 * chassis->feedback.wheelSpeed[2] + SQRT2 * chassis->feedback.wheelSpeed[3]) *
                            0.25f;
    chassis->feedback.v.z = (-SQRT2 * chassis->feedback.wheelSpeed[0] - SQRT2 * chassis->feedback.wheelSpeed[1] -
                             SQRT2 * chassis->feedback.wheelSpeed[2] - SQRT2 * chassis->feedback.wheelSpeed[3]) *
                            0.25f;

#else
    fp32 opsSin;
    fp32 opsCos;
    arm_sin_cos_f32(ops.yaw, &opsSin, &opsCos);
    chassis->feedback.v.x = opsCos * ops.vx - opsSin * ops.vy;
    chassis->feedback.v.y = opsCos * ops.vy + opsSin * ops.vx;
    chassis->feedback.v.z = ops.w;
    //TODO: 滤波融合
#endif
#ifndef WHEEL_ODOM
    chassis->feedback.v.expand = (SQRT2 * chassis->feedback.wheelSpeed[0] - SQRT2 * chassis->feedback.wheelSpeed[1] +
                                  SQRT2 * chassis->feedback.wheelSpeed[2] - SQRT2 * chassis->feedback.wheelSpeed[3]) *
                                 0.25f;
    chassis->feedback.p.x = ops.x / 1000.0f;
    chassis->feedback.p.y = ops.y / 1000.0f;
    chassis->feedback.p.z = DEG2RAD(ops.yaw);
#else
    //TODO: 里程计
#endif
    chassis->feedback.updateTime = xTaskGetTickCount();
  }
}

/**
  * @brief    底盘逆运动学解算
  * 
  * @param    chassis   底盘结构体
  */
void chassisIK(DrChassis *chassis) {
  if (chassis->feedbackMode == CHASSIS_FEEDBACK_WHEEL) {
    chassis->out[0] = (-0.5f * SQRT2 * chassis->set.x - 0.5f * SQRT2 * chassis->set.y -
                       chassis->set.z * chassis->wheelCenterDistance + 0.5f * SQRT2 * chassis->set.expand);
    chassis->out[1] = (0.5f * SQRT2 * chassis->set.x - 0.5f * SQRT2 * chassis->set.y -
                       chassis->set.z * chassis->wheelCenterDistance - 0.5f * SQRT2 * chassis->set.expand);
    chassis->out[2] = (0.5f * SQRT2 * chassis->set.x + 0.5f * SQRT2 * chassis->set.y -
                       chassis->set.z * chassis->wheelCenterDistance + 0.5f * SQRT2 * chassis->set.expand);
    chassis->out[3] = (-0.5f * SQRT2 * chassis->set.x + 0.5f * SQRT2 * chassis->set.y -
                       chassis->set.z * chassis->wheelCenterDistance - 0.5f * SQRT2 * chassis->set.expand);
    for (uint8_t i = 0; i < 4; i++) {
      // / 60 * 2 * PI *WHEEL_RADIUS
      chassis->out[i] *= 60;
      chassis->out[i] /= 2 * PI * WHEEL_RADIUS;
    }
  } else if (chassis->feedbackMode == CHASSIS_FEEDBACK_OPS_YAW) {
    fp32 wzSet      = controllerUpdate((Controller*)&chassisYawPid, &chassisLockYaw, &ops.yaw, NULL);
    chassis->out[0] = (-0.5f * SQRT2 * chassis->set.x - 0.5f * SQRT2 * chassis->set.y - wzSet * chassis->wheelCenterDistance +
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[1] = (0.5f * SQRT2 * chassis->set.x - 0.5f * SQRT2 * chassis->set.y - wzSet * chassis->wheelCenterDistance -
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[2] = (0.5f * SQRT2 * chassis->set.x + 0.5f * SQRT2 * chassis->set.y - wzSet * chassis->wheelCenterDistance +
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[3] = (-0.5f * SQRT2 * chassis->set.x + 0.5f * SQRT2 * chassis->set.y - wzSet * chassis->wheelCenterDistance -
                       0.5f * SQRT2 * chassis->set.expand);
  } else if (chassis->feedbackMode == CHASSIS_FEEDBACK_OPS_FULL) {
    fp32 cos;
    fp32 sin;
    arm_sin_cos_f32(ops.yaw, &sin, &cos);
    fp32 vxSet;
    fp32 vySet;
    fp32 wzSet;
    fp32 ref        = cos * ops.x + sin * ops.y;
    fp32 set        = cos * chassisLockX + sin * chassisLockY;
    vxSet           = controllerUpdate((Controller*)&chassisXPid, &set, &ref, NULL);
    ref             = -sin * ops.x + cos * ops.y;
    set             = -sin * chassisLockX + cos * chassisLockY;
    vySet           = controllerUpdate((Controller*)&chassisYPid, &set, &ref, NULL);
    wzSet           = controllerUpdate((Controller*)&chassisYawPid, &chassisLockYaw, &ops.yaw, NULL);
    chassis->out[0] = (-0.5f * SQRT2 * vxSet - 0.5f * SQRT2 * vySet - wzSet * chassis->wheelCenterDistance +
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[1] = (0.5f * SQRT2 * vxSet - 0.5f * SQRT2 * vySet - wzSet * chassis->wheelCenterDistance -
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[2] = (0.5f * SQRT2 * vxSet + 0.5f * SQRT2 * vySet - wzSet * chassis->wheelCenterDistance +
                       0.5f * SQRT2 * chassis->set.expand);
    chassis->out[3] = (-0.5f * SQRT2 * vxSet + 0.5f * SQRT2 * vySet - wzSet * chassis->wheelCenterDistance -
                       0.5f * SQRT2 * chassis->set.expand);
    for (uint8_t i = 0; i < 4; i++) {
      // / 60 * 2 * PI *WHEEL_RADIUS
      chassis->out[i] *= 60;
      chassis->out[i] /= 2 * PI * WHEEL_RADIUS;
    }
  }
}

void chassisExecute(DrChassis *chassis) {
  if (chassis->feedbackMode == CHASSIS_FEEDBACK_WHEEL || chassis->feedbackMode == CHASSIS_FEEDBACK_OPS_FULL) {
    for (uint8_t i = 0; i < 4; i++) {
      Motor_SetSpeed(chassis->motor[i], chassis->out[i], 4);
      //Motor_AltControl(chassis->motor[i + 4], 4);
    }
  }
}

/**
  * @brief    前向运动学速度反馈控制
  * 
  * @note     借用了底盘四个电机的PID 使用该模式时应该先调用PID_Clear()
  * 
  */
MotorInstructType chassisFKSpeedFeedbackCtrl(Motor *motor, Controller *controller, void *param) {
  (void)controller;
  if (motor == drChassis.motor[0]) {
    chassisFeedback(&drChassis);
    fp32 axSet =
        controllerUpdate((Controller*)&((RM_Motor *)(drChassis.motor[0]))->speed_pid, &drChassis.set.x, &drChassis.feedback.v.x, NULL);
    fp32 aySet =
        controllerUpdate((Controller*)&((RM_Motor *)(drChassis.motor[1]))->speed_pid, &drChassis.set.y, &drChassis.feedback.v.y, NULL);
    fp32 azSet =
        controllerUpdate((Controller*)&((RM_Motor *)(drChassis.motor[2]))->speed_pid, &drChassis.set.z, &drChassis.feedback.v.z, NULL);
    fp32 aExpandSet     = drChassis.set.expand + controllerUpdate((Controller*)&((RM_Motor *)(drChassis.motor[3]))->speed_pid,
                                                              &drChassis.set.expand, &drChassis.feedback.v.expand, NULL);
    drChassis.out[0]    = (-0.5f * SQRT2 * axSet - 0.5f * SQRT2 * aySet - azSet + 0.5f * SQRT2 * aExpandSet);
    drChassis.out[1]    = (0.5f * SQRT2 * axSet - 0.5f * SQRT2 * aySet - azSet - 0.5f * SQRT2 * aExpandSet);
    drChassis.out[2]    = (0.5f * SQRT2 * axSet + 0.5f * SQRT2 * aySet - azSet + 0.5f * SQRT2 * aExpandSet);
    drChassis.out[3]    = (-0.5f * SQRT2 * axSet + 0.5f * SQRT2 * aySet - azSet - 0.5f * SQRT2 * aExpandSet);
    motor->instruct.set = drChassis.out[0];
    return INSTRUCT_CURRENT;
  }
  for (uint8_t i = 1; i < 4; i++) {
    if (motor == drChassis.motor[i]) {
      motor->instruct.set = drChassis.out[i];
    }
  }
  return INSTRUCT_CURRENT;
}

/**
  * @brief    双电机控制
  * 
  * @param    motor     
  * @param    controller
  * @param    param     
  * @return   MotorInstructType
  */
MotorInstructType chassisDoubleMotorCtrl(Motor *motor, Controller *controller, void *param) {
  (void)controller;
  //RM_Motor* mirror_motor=(RM_Motor *)(param);
  motor->instruct.set = -((RM_Motor *)(param))->set_current;
  return INSTRUCT_CURRENT;
}

void chassis_task0() {
  vTaskDelay(500);
  //找电机
  for (;;) {

    drChassis.motor[0] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 0);
    drChassis.motor[1] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 1);
    drChassis.motor[2] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 2);
    drChassis.motor[3] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 1);
    // drChassis.motor[4] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 3);
    // drChassis.motor[5] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 1);
    // drChassis.motor[6] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN2, 0);
    // drChassis.motor[7] = CAN_Find_Motor(RM_MOTOR, INTERNAL_CAN1, 2);
    for (uint8_t i = 0; i < 4; i++) {
      if (drChassis.motor[i] == NULL) {
        vTaskDelay(20);
        continue;
      } else {
        RM_Motor_setAsM3508(drChassis.motor[i]);
      }
    }
    break;
  }
  //跟随电机必须比主动电机逻辑靠后
  for (uint8_t i = 0; i < 4; i++) {
    //Motor_SetAltController(drChassis.motor[i + 4], NULL, (void *)drChassis.motor[i], chassisDoubleMotorCtrl);
    Motor_SetSpeedPID(drChassis.motor[i], &chassisMotorSpeedPidParam);
    Motor_SetAnglePID(drChassis.motor[i], &chassisMotorAnglePidParam);
  }
  drChassis.chassisFsm.currentState = &chassisEase;
  drChassis.wheelCenterDistance=WHEEL_CENTER_DISTANCE_NARROW;
  PID_ControllerInit(&chassisXPid, &chassisXConstrain, &chassisXPidParam, 0.004);
  PID_ControllerInit(&chassisYPid, &chassisYConstrain, &chassisYPidParam, 0.004);
  PID_ControllerInit(&chassisYawPid, &chassisYawConstrain, &chassisYawPidParam, 0.004);
  for (;;) {
    //finiteStateMachine(drChassis.state);
    Sbus_lpf();
    if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
      chassisFeedback(&drChassis);
      finiteStateMachine(&drChassis.chassisFsm);
      // chassisIK(&drChassis);
      // chassisExecute(&drChassis);
    } else {
      for (uint8_t i = 0; i < 4; i++) {
        Motor_SetCurrent(drChassis.motor[i], 0, 10);
      }
    }
    vTaskDelay(2);
  }
}

static void chassisLockExe(StateMachine *fsm, void *param) {
  drChassis.feedbackMode    = CHASSIS_FEEDBACK_OPS_FULL;
  drChassis.set.expandForce = 0.5f * expandFlag;
  drChassis.set.expand      = 0.5f * expandFlag;
  chassisIK(&drChassis);
  chassisExecute(&drChassis);
}

static void chassisMoveSwitchIn(StateMachine *fsm, void *param) { chassisLockYaw = ops.yaw; }

static State *chassisMoveTrans(StateMachine *fsm, void *param) {
  if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
    if (sbus.real.channel[SBUS_SB_CHANNEL] > 0.8f) {
      return &chassisLockMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] == 0) {
      return &chassisMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] < -0.8f) {
      return &chassisLock;
    }
  }
  return &chassisEase;
}

static uint16_t stallTime;

static void chassisMoveExe(StateMachine *fsm, void *param) {
  drChassis.set.x           = 5*sbus.filtered.channel[SBUS_RY_CHANNEL];
  drChassis.set.y           = -5*sbus.filtered.channel[SBUS_RX_CHANNEL];
  drChassis.set.z           = 5*sbus.filtered.channel[SBUS_LX_CHANNEL];
  drChassis.set.expandForce = 0.2f * expandFlag;
  drChassis.set.expand      = 0.2f * expandFlag + 5*sbus.filtered.channel[SBUS_LY_CHANNEL];
  drChassis.feedbackMode    = CHASSIS_FEEDBACK_WHEEL;
  chassisIK(&drChassis);
  chassisExecute(&drChassis);
  if (sbus.real.channel[SBUS_LY_CHANNEL] > 0.6f) {
    stallTime++;
    if (stallTime > 250) {
      stallTime  = 0;
      expandFlag = 1;
    }
  } else if (sbus.filtered.channel[SBUS_LY_CHANNEL] < -0.6f) {
    stallTime++;
    if (stallTime > 250) {
      stallTime  = 0;
      expandFlag = -1;
    }
  } else {
    stallTime = 0;
  }
}


void chassisLockMoveSwitchIn(StateMachine *fsm, void *param) {
  chassisLockX = ops.x;
  chassisLockY = ops.y;
  chassisLockY = ops.yaw;
}

static State *chassisLockMoveTrans(StateMachine *fsm, void *param) {
  if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
    if (sbus.real.channel[SBUS_SB_CHANNEL] > 0.8f) {
      return &chassisLockMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] == 0) {
      return &chassisMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] < -0.8f) {
      return &chassisLock;
    }
  }
  return &chassisEase;
}

void chassisLockMoveExe(StateMachine *fsm, void *param) {
  drChassis.feedbackMode = CHASSIS_FEEDBACK_OPS_FULL;
  fp32 sin;
  fp32 cos;
  arm_sin_cos_f32(ops.yaw, &sin, &cos);
  chassisLockX += -sin * sbus.filtered.channel[SBUS_RY_CHANNEL] - cos * sbus.filtered.channel[SBUS_RX_CHANNEL];
  chassisLockY += cos * sbus.filtered.channel[SBUS_RY_CHANNEL] - sin * sbus.filtered.channel[SBUS_RX_CHANNEL];
  chassisLockY += sbus.filtered.channel[SBUS_LX_CHANNEL];
  drChassis.set.expandForce = 0.1f * expandFlag;
  drChassis.set.expand      = 0.1f * expandFlag;
  chassisIK(&drChassis);
  chassisExecute(&drChassis);
}

void   chassisEaseSwitchIn(StateMachine *fsm, void *param) { expandFlag = -1; }
State *chassisEaseTrans(StateMachine *fsm, void *param) {
  if (sbus.real.channel[SBUS_SA_CHANNEL] > 0.8f) {
    if (sbus.real.channel[SBUS_SB_CHANNEL] > 0.8f) {
      return &chassisLockMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] == 0) {
      return &chassisMove;
    } else if (sbus.real.channel[SBUS_SB_CHANNEL] < -0.8f) {
      return &chassisLock;
    }
  }
  return &chassisEase;
}
void chassisEaseExe(StateMachine *fsm, void *param) {
  for (uint8_t i = 0; i < 8; i++) {
    Motor_SetCurrent(drChassis.motor[i], 0, 10);
  }
}
