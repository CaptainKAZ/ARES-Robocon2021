/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     fsm.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    有限状态机
  * @version  0.1
  * @date     2021-07-11
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef FSM_H
#define FSM_H
#include "main.h"

typedef struct StateMachine StateMachine;
typedef struct State State;

typedef struct State{
  /**
    * @brief    该状态切入
    * 
    */
  void (*onSwitchIn)(StateMachine *fsm,void* param);
  /**
    * @brief    该状态切出
    * 
    */
  void (*onSwitchOut)(StateMachine* fsm,void* param);
  /**
    * @brief    该状态执行
    * 
    */
  void (*stateExecute)(StateMachine *fsm,void* param);
  /**
    * @brief    状态转移判断
    * 
    */
  State *(*stateTransistion)(StateMachine *fsm,void* param);
  /**
    * @brief    该状态内部的参数
    * 
    */
  void *   param;
} State;

typedef struct StateMachine{
  /**
    * @brief    当前状态组
    * 
    */
  State *currentState;
  /**
    * @brief    状态机邮箱 用于储存状态转移请求
    * 
    */
  int32_t mailBox;
  /**
    * @brief    状态时间
    * 
    */
  uint32_t stateTime;
} StateMachine;

void finiteStateMachine(StateMachine *StateMachine);

#endif
