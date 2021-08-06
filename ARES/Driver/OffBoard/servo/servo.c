/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     servo.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    舵机控制
  * @version  0.1
  * @date     2021-07-15
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "servo.h"
#include "string.h"

Servo servo;

uint8_t Interboard_servoRxHook(uint8_t *buf, uint8_t len) {
  if (len == 11) {
#ifdef MASTER_BOARD
    if (buf[0] == SERVO_SET_POS || buf[0] == SERVO_GET_POS) {
      servo.updated[buf[1]]   = 1;
      servo.actualPos[buf[1]] = buf[2] << 8 | buf[3];
      servo.exist[buf[1]]     = buf[4];
      servo.numOfServo        = buf[5];
    } else if (buf[0] == SERVO_GET_EXIST) {
      for (uint8_t i = 0; i < 10; i++) {
        servo.exist[i] = buf[i + 1];
      }
    } else {
      return 0;
    }
#elif defined SLAVE_BOARD
    uint8_t servoBuf[11];
    memset(servoBuf, 0, 11);
    if (buf[0] == SERVO_SET_POS) {
      servoBuf[0]       = SERVO_SET_POS;
      servoBuf[1]       = buf[1];
      servoBuf[2]       = servo.actualPos[buf[1]] >> 8;
      servoBuf[3]       = servo.actualPos[buf[1]] | 0xFF;
      servoBuf[4]       = servo.exist[buf[i]];
      servoBuf[5]       = servo.numOfServo;
      servo.expectedPos = buf[2] << 8 | buf[3];
    } else if (buf[0] == SERVO_GET_POS) {
      servoBuf[0] = SERVO_SET_POS;
      servoBuf[1] = buf[1];
      servoBuf[2] = servo.actualPos[buf[1]] >> 8;
      servoBuf[3] = servo.actualPos[buf[1]] | 0xFF;
      servoBuf[4] = servo.exist[buf[i]];
      servoBuf[5] = servo.numOfServo;
    } else if (buf[0] == SERVO_GET_EXIST) {
      servoBuf[0] = SERVO_GET_EXIST;
      for (uint8_t i = 0; i < 10; i++) {
        servoBuf[i + 1] = servo.exist[i];
      }
    } else {
      return 0;
    }
    Interboard_tx(INTERBOARDMSG_SERVO, 11, servoBuf);
#endif
    return 1;
  }
  return 0;
}
#ifdef MASTER_BOARD
void Servo_setPos (uint8_t id,uint16_t pos){
  uint8_t servoBuf[11];
  memset(servoBuf, 0, 11);
  servo.expectedPos[id] = pos;
  servoBuf[0] = SERVO_SET_POS;
  servoBuf[1] = id;
  servoBuf[2] = pos >> 8;
  servoBuf[3] = pos & 0xFF;
  servo.updated[id] = 0;
  Interboard_tx(INTERBOARDMSG_SERVO, 11, servoBuf);
}

void Servo_getPosNonBlocking(uint8_t id) {
  uint8_t servoBuf[11];
  memset(servoBuf, 0, 11);
  servoBuf[0] = SERVO_GET_POS;
  servoBuf[1] = id;
  Interboard_tx(INTERBOARDMSG_SERVO, 11, servoBuf);
  servo.updated[id] = 0;
}

uint16_t Servo_getPosBlocking(uint8_t id) {
  uint8_t servoBuf[11];
  memset(servoBuf, 0, 11);
  servoBuf[0] = SERVO_GET_POS;
  servoBuf[1] = id;
  servo.updated[id] = 0;
  Interboard_tx(INTERBOARDMSG_SERVO, 11, servoBuf);
  while(servo.updated[id]==0)
    ;
  return servo.actualPos[id];
}

void Servo_getExistNonBlocking(){
  uint8_t servoBuf[11];
  memset(servoBuf, 0, 11);
  servoBuf[0] = SERVO_GET_EXIST;
  Interboard_tx(INTERBOARDMSG_SERVO, 11, servoBuf);
}
#endif
