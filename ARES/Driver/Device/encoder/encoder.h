/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     encoder.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    CAN多圈编码器驱动程序
  * @version  0.1
  * @date     2021-03-05
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#include "can_comm.h"

typedef enum {
  ENCODER_READ              = 0x01, //读取编码器值
  ENCODER_SET_ID            = 0x02, //设置编码器ID
  ENCODER_SET_BAUDRATE      = 0x03, //设置CAN通信波特率
  ENCODER_SET_MODE          = 0x04, //设置编码器模式
  ENCODER_SET_FEEDBACK_TIME = 0x05, //设置自动回传时间 建议大于50ms
  ENCODER_ZERO              = 0x06, //设置当前位置为编码器原点
} CAN_ENDCODER_COMMAND;

typedef struct{
  int32_t value;
  int32_t updateTime;
} CanEncoder;

#define ENCODER_BROADCAST_ID 0x00
#define ENCODER_MASTER_ID 0x01

extern CanEncoder canEncoder[2][4];

extern void Encoder_setFeedbackTime(CAN_Device device, uint8_t id, uint16_t ms);
extern void Encoder_readValue(CAN_Device device, uint8_t id);
extern void Encoder_setMode(CAN_Device device, uint8_t id, uint8_t mode);
extern void Encoder_zero(CAN_Device device, uint8_t id);

#endif
