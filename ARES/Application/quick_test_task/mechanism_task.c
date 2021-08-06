/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mechanism_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    
  * @version  0.1
  * @date     2021-07-20
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "motor.h"
#include "fsm.h"
StateMachine mechanismFsm;

static mechanismEaseExe(StateMachine *fsm, void *param);
static mechanismEaseTrans(StateMachine *fsm, void *param);
State mechanismEase = {.param            = NULL,
                       .onSwitchIn       = NULL,
                       .onSwitchOut      = NULL,
                       .stateExecute     = mechanismEaseExe,
                       .stateTransistion = mechanismEaseTrans};

static uint32_t mechanismCalibrationReg = 0;
static mechanismCalibrationExe(StateMachine *fsm, void *param);
static mechanismCalibrationTrans(StateMachine *fsm, void *param);
static mechanismCalibrationSwitchIn(StateMachine *fsm, void *param);
State mechanismCalibration = {.param            = &mechanismCalibrationReg,
                              .onSwitchIn       = mechanismCalibrationExe,
                              .onSwitchOut      = NULL,
                              .stateExecute     = mechanismCalibrationExe,
                              .stateTransistion = mechanismCalibrationTrans};

static mechanismStandByExe(StateMachine *fsm, void *param);
static mechanismStandByTrans(StateMachine *fsm, void *param);
State mechanismStandBy = {.param            = NULL,
                              .onSwitchIn       = NULL,
                              .onSwitchOut      = NULL,
                              .stateExecute     = mechanismStandByExe,
                              .stateTransistion = mechanismStandByTrans};

static mechanismShootExe(StateMachine *fsm, void *param);
static mechanismShootTrans(StateMachine *fsm, void *param);
State mechanismShoot = {.param            = NULL,
                        .onSwitchIn       = NULL,
                        .onSwitchOut      = NULL,
                        .stateExecute     = mechanismShootExe,
                        .stateTransistion = mechanismShootTrans};

static mechanismPickExe(StateMachine *fsm, void *param);
static mechanismPickTrans(StateMachine *fsm, void *param);
State mechanismPick = {.param            = NULL,
                       .onSwitchIn       = NULL,
                       .onSwitchOut      = NULL,
                       .stateExecute     = mechanismPickExe,
                       .stateTransistion = mechanismPickTrans};

Motor *motor[5];

void mechanism_task(){
  
}
