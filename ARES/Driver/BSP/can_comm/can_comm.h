/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     can_comm.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    CAN统一收发函数，后期可拓展支持外置CAN收发器
  * @version  0.1
  * @date     2021-02-20
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef CAN_COMM_H
#define CAN_COMM_H
#include "main.h"
#include "can.h"

typedef enum {
  INTERNAL_CAN1=0,
  INTERNAL_CAN2,
} CAN_Device;

typedef enum {
  CAN_FRAME_STD,
  CAN_FRAME_EXT,
} CAN_FrameType;

typedef struct {
  CAN_Device    device;
  CAN_FrameType type;
  uint32_t      id;
  uint8_t *     data;
  uint8_t       len;
} CAN_Frame;

#define ENCODER_RX 0x01
#define RM_MOTOR_RX_L 0x201
#define RM_MOTOR_RX_H 0x208
#define RMD_MOTOR_RX_L 0x141
#define RMD_MOTOR_RX_H 0x148

extern void CAN_RxHook(CAN_Frame *frame);
extern void CAN_Tx(CAN_Frame *frame);

extern void CAN_Start(CAN_Device device);
#endif
