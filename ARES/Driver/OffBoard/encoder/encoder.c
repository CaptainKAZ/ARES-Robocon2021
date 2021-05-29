/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     encoder.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    CAN多圈编码器驱动程序
  * @version  0.1
  * @date     2021-03-05
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "encoder.h"
#include "cmsis_os.h"

CanEncoder canEncoder[2][4];

void Encoder_setFeedbackTime(CAN_Device device, uint8_t id, uint16_t us) {
  uint8_t   data[5];
  CAN_Frame frame;
  frame.id     = id;
  data[0]      = 0x05;
  data[1]      = id & 0xFF;
  data[2]      = ENCODER_SET_FEEDBACK_TIME;
  data[3]      = us & 0xFF;
  data[4]      = (us >> 8) & 0xFF;
  frame.data   = data;
  frame.device = device;
  frame.len    = 5;
  frame.type   = CAN_FRAME_STD;
  CAN_Tx(&frame);
}

void Encoder_setId(CAN_Device device, uint8_t oldId, uint16_t newId) {
  uint8_t   data[4];
  CAN_Frame frame;
  frame.id     = oldId;
  data[0]      = 0x04;
  data[1]      = oldId & 0xFF;
  data[2]      = ENCODER_SET_ID;
  data[3]      = newId & 0xFF;
  frame.data   = data;
  frame.device = device;
  frame.len    = 4;
  frame.type   = CAN_FRAME_STD;
  CAN_Tx(&frame);
}

void Encoder_readValue(CAN_Device device, uint8_t id) {
  uint8_t   data[4];
  CAN_Frame frame;
  frame.id     = id;
  data[0]      = 0x04;
  data[1]      = id;
  data[2]      = ENCODER_READ;
  data[3]      = 0x00;
  frame.data   = data;
  frame.len    = 4;
  frame.device = device;
  frame.type   = CAN_FRAME_STD;
  CAN_Tx(&frame);
}

void Encoder_setMode(CAN_Device device, uint8_t id, uint8_t mode) {
  uint8_t   data[4];
  CAN_Frame frame;
  frame.id     = id;
  data[0]      = 0x04;
  data[1]      = id;
  data[2]      = ENCODER_SET_MODE;
  data[3]      = mode;
  frame.data   = data;
  frame.len    = 4;
  frame.device = device;
  frame.type   = CAN_FRAME_STD;
  CAN_Tx(&frame);
}

void Encoder_zero(CAN_Device device, uint8_t id) {
  uint8_t   data[4];
  CAN_Frame frame;
  frame.id     = id;
  data[0]      = 0x04;
  data[1]      = id;
  data[2]      = ENCODER_ZERO;
  data[3]      = 0x00;
  frame.data   = data;
  frame.len    = 4;
  frame.device = device;
  frame.type   = CAN_FRAME_STD;
  CAN_Tx(&frame);
}

void Encoder_RxHook(CAN_Frame *frame) {
  if (frame->data[2] == 0x01) {
      canEncoder[frame->device][frame->id - 2].value = frame->data[3] | frame->data[4] << 8 | frame->data[5] << 16 | frame->data[6] << 24;
      canEncoder[frame->device][frame->id - 2].updateTime = xTaskGetTickCount();
  }
}
