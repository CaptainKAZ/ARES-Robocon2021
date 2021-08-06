/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     servo.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    舵机控制
  * @version  0.1
  * @date     2021-07-15
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef SERVO_H
#define SERVO_H
#include "interboard_spi.h"
#include "main.h"

typedef struct {
  uint16_t actualPos[10];
  uint16_t expectedPos[10];
  uint8_t  exist[10];
  uint8_t  numOfServo;
  uint8_t  updated[10];
} Servo;

typedef enum {
  SERVO_SET_POS,
  SERVO_GET_POS,
  SERVO_GET_EXIST,
} servoOperation;

#ifdef MASTER_BOARD
extern void     Servo_setPos(uint8_t id, uint16_t pos);
extern void     Servo_getPosNonBlocking(uint8_t id);
extern uint16_t Servo_getPosBlocking(uint8_t id);
extern void     Servo_getExistNonBlocking(void);
#elif defined   SALVE_BOARD
#endif

extern Servo servo;

#endif
