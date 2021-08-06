/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     fsm.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    有限状态机
  * @version  0.1
  * @date     2021-07-11
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "fsm.h"
#include "cmsis_os.h"

void finiteStateMachine(StateMachine *stateMachine) {
  if (stateMachine) {
    if (stateMachine->currentState->stateExecute) {
      stateMachine->currentState->stateExecute(stateMachine, stateMachine->currentState->param);
    }
    if (stateMachine->currentState->stateTransistion) {
      State *nextState = stateMachine->currentState->stateTransistion(stateMachine, stateMachine->currentState->param);
      if (nextState != stateMachine->currentState) {
        if (stateMachine->currentState->onSwitchOut) {
          stateMachine->currentState->onSwitchOut(stateMachine, stateMachine->currentState->param);
        }
        stateMachine->stateTime = xTaskGetTickCount();
        if (nextState->onSwitchIn) {
          nextState->onSwitchIn(stateMachine, stateMachine->currentState->param);
        }
      }
      stateMachine->currentState = nextState;
    }
  }
}
