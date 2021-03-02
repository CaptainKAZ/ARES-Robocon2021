/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     VESC_motor.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    VESC电机驱动
  * @version  0.1
  * @date     2021-03-01
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "VESC_motor.h"
#include "cmsis_os.h"
#include "semphr.h"

static VESC_Motor vesc_motor[8];

static SemaphoreHandle_t VESC_Motor_Mutex = NULL;
static StaticSemaphore_t VESC_Motor_MutexBuffer;

#define MOTOR ((Motor *)self)
#define VESC ((VESC_Motor *)self)

static void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

static void VESC_Motor_Command(CAN_Device *device, uint32_t eid, uint8_t *data, uint8_t len) {
  if (len > 8) {
    len = 8;
  }
  CAN_Frame frame;
  frame.device = device;
  frame.data   = data;
  frame.id     = eid;
  frame.type   = CAN_FRAME_EXT;
  CAN_Tx(&frame);
}

void VESC_Motor_Init(VESC_Motor *self, CAN_Device device, uint8_t id) {
  for (uint8_t i = 0; i < sizeof(VESC_Motor); i++) {
    ((uint8_t *)self)[i] = 0;
  }
  MOTOR->info.device = device;
  MOTOR->info.id     = id;
  MOTOR->info.type   = VESC_MOTOR;
}

void VESC_Motor_RxHook(CAN_Frame *frame) {
  uint8_t id         = (frame->id) & 0xFF;
  fp32    last_angle = 0;
  uint8_t empty      = 255;
  for (uint8_t i = 0; i < 8; i++) {
    if (vesc_motor[i].general.info.type == VESC_MOTOR) {
      if (vesc_motor[i].general.info.device == frame->device && vesc_motor->general.info.id == id)
        switch (frame->id >> 8) {
        case CAN_PACKET_STATUS:
          vesc_motor[i].status1_timestamp = xTaskGetTickCount();
          vesc_motor[i].general.status.speed =
              (frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3]);
          vesc_motor[i].general.status.current = (frame->data[4] << 8 | frame->data[5]) / 10.0f;
          break;
        case CAN_PACKET_STATUS_4:
          vesc_motor[i].status4_timestamp          = xTaskGetTickCount();
          last_angle                               = vesc_motor->general.status.angle;
          vesc_motor[i].general.status.angle       = DEG2RAD(((frame->data[4] << 8 | frame->data[5]) / 50.0f));
          vesc_motor[i].general.status.temperature = (frame->data[2] << 8 | frame->data[3]) / 10.0f;
          if (vesc_motor[i].general.status.angle - last_angle > PI) {
            vesc_motor[i].cumulative_turn--;
          } else if (vesc_motor[i].general.status.angle - last_angle < -PI) {
            vesc_motor[i].cumulative_turn++;
          }
          break;
        }
      return;
    } else {
      if (empty > i) {
        empty = i;
      }
    }
  }
  if (empty != 255) {
    VESC_Motor_Init(&vesc_motor[empty], frame->device, id);
    switch (frame->id >> 8) {
    case CAN_PACKET_STATUS:
      vesc_motor[empty].status1_timestamp = xTaskGetTickCount();
      vesc_motor[empty].general.status.speed =
          (frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3]);
      vesc_motor[empty].general.status.current = (frame->data[4] << 8 | frame->data[5]) / 10.0f;
      break;
    case CAN_PACKET_STATUS_4:
      vesc_motor[empty].status4_timestamp          = xTaskGetTickCount();
      last_angle                                   = vesc_motor->general.status.angle;
      vesc_motor[empty].general.status.angle       = DEG2RAD(((frame->data[4] << 8 | frame->data[5]) / 50.0f));
      vesc_motor[empty].general.status.temperature = (frame->data[2] << 8 | frame->data[3]) / 10.0f;

      break;
    }
  }
}

void VESC_Motor_Zero(VESC_Motor *self) {
  if (VESC_Motor_Mutex = NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  VESC->zero            = MOTOR->status.angle;
  VESC->cumulative_turn = 0;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetSpeed(VESC_Motor *self, fp32 rpm) {
  if (VESC_Motor_Mutex = NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type = INSTRUCT_SPEED;
  MOTOR->instruct.set  = rpm;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetAngle(VESC_Motor *self, fp32 rad) {
  if (VESC_Motor_Mutex = NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type = INSTRUCT_ANGLE;
  MOTOR->instruct.set  = rad;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetAlternative(VESC_Motor *self, Controller *alt_controller,
                             fp32 (*alt_controller_update)(Motor *motor, Controller *controller)) {
  if (VESC_Motor_Mutex = NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type      = INSTRUCT_ALTERNATIVE;
  VESC->alt_controller        = alt_controller;
  VESC->alt_controller_update = alt_controller_update;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_Execute() {
  if (VESC_Motor_Mutex = NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  int32_t send_index = 0;
  uint8_t buffer[4];
  for (uint8_t i = 0; i < 8; i++) {
    if (vesc_motor[i].general.info.type == VESC_MOTOR) {
      switch (vesc_motor[i].general.instruct.type) {
      case INSTRUCT_CURRENT:
        buffer_append_int32(buffer, (int32_t)(vesc_motor[i].general.instruct.set), &send_index);
        VESC_Motor_Command(vesc_motor[i].general.info.device,
                           vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
        break;
      case INSTRUCT_SPEED:
        buffer_append_int32(buffer, (int32_t)(vesc_motor[i].general.instruct.set), &send_index);
        VESC_Motor_Command(vesc_motor[i].general.info.device,
                           vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
        break;
      case INSTRUCT_ANGLE:
        buffer_append_int32(buffer, (int32_t)(RAD2DEG(vesc_motor[i].general.instruct.set) * 1000000.0f), &send_index);
        VESC_Motor_Command(vesc_motor[i].general.info.device,
                           vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
        break;
      case INSTRUCT_ALTERNATIVE:
        buffer_append_int32(
            buffer, (int32_t)(vesc_motor[i].alt_controller_update(&vesc_motor[i], vesc_motor[i].alt_controller)), &send_index);
        VESC_Motor_Command(vesc_motor[i].general.info.device,
                           vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
        break;
      }
    }
  }
  xSemaphoreGive(VESC_Motor_Mutex);
}
