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
#include "vesc_motor.h"
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

static void VESC_Motor_Command(CAN_Device device, uint32_t eid, uint8_t *data, uint8_t len) {
  if (len > 8) {
    len = 8;
  }
  CAN_Frame frame;
  frame.device = device;
  frame.data   = data;
  frame.id     = eid;
  frame.len     = len;
  frame.type   = CAN_FRAME_EXT;
  CAN_Tx(&frame);
}

static void VESC_Motor_Init(VESC_Motor *self, CAN_Device device, uint8_t id) {
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
  int8_t  index      = 127; // Positive means found; negative means the empty one; 127 means full

  // Find motor
//  for (uint8_t i = 0; i < 8; i++) {
//    if (vesc_motor->general.info.type == VESC_MOTOR) {
//      if (vesc_motor[i].general.info.device == frame->device && vesc_motor[i].general.info.id == id) {
//        index = i;
//        break;
//      }
//    } else {
//      index = -i;
//    }
//  }
//  if (index == 127) {
//    return;
//  } else if (index < 0) {
//    index = -index;
//    if ((frame->id >> 8) == CAN_PACKET_STATUS_4) {
//      VESC_Motor_Init(&vesc_motor[index], frame->device, id);
//      vesc_motor[index].general.status.angle = DEG2RAD(((frame->data[4] << 8 | frame->data[5]) / 50.0f))-vesc_motor[index].general.status.zero;
//    } else {
//      return;
//    }
//  }
  index= id;
  if ((frame->id >> 8) == CAN_PACKET_STATUS_4&&vesc_motor[index].general.info.type!=VESC_MOTOR) {
      VESC_Motor_Init(&vesc_motor[index], frame->device, id);
      vesc_motor[index].general.status.angle = DEG2RAD(((frame->data[4] << 8 | frame->data[5]) / 50.0f))-vesc_motor[index].general.status.zero;
    } else if(vesc_motor[index].general.info.type!=VESC_MOTOR) {
      VESC_Motor_Init(&vesc_motor[index], frame->device, id);
      return;
    }
  switch (frame->id >> 8) {
  case CAN_PACKET_STATUS:
    vesc_motor[index].status1_timestamp = xTaskGetTickCount();
    vesc_motor[index].general.status.speed =
        (frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3]);
    vesc_motor[index].general.status.current = (frame->data[4] << 8 | frame->data[5]) / 10.0f;
    break;
  case CAN_PACKET_STATUS_4:
    vesc_motor[index].status4_timestamp          = xTaskGetTickCount();
    last_angle                                   = vesc_motor->general.status.angle;
    vesc_motor[index].general.status.angle       = DEG2RAD(((frame->data[4] << 8 | frame->data[5]) / 50.0f));
    vesc_motor[index].general.status.temperature = (frame->data[2] << 8 | frame->data[3]) / 10.0f;
    if (vesc_motor[index].general.status.angle - last_angle > PI) {
      vesc_motor[index].general.status.cumulativeTurn--;
    } else if (vesc_motor[index].general.status.angle - last_angle < -PI) {
      vesc_motor[index].general.status.cumulativeTurn++;
    }
    break;
  default: break;
  }
}

void VESC_Motor_Zero(Motor *self) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->status.cumulativeTurn = 0;
  MOTOR->status.zero           = MOTOR->status.angle;
  MOTOR->status.angle          = 0;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_CURRENT;
  MOTOR->instruct.set     = mA;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_SPEED;
  MOTOR->instruct.set     = rpm;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ANGLE;
  MOTOR->instruct.set     = rad;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                                 MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                            void *param)) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->alt_controller        = alt_controller;
  MOTOR->alt_controller_update = alt_controller_update;
  MOTOR->alt_controller_param  = param;
  xSemaphoreGive(VESC_Motor_Mutex);
}

void VESC_Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != VESC_MOTOR) {
    return;
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ALTERNATIVE;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(VESC_Motor_Mutex);
}

Motor *VESC_Motor_Find(CAN_Device device, uint8_t id) {
  for (uint8_t i = 0; i < 8; i++) {
    if (vesc_motor[i].general.info.device == device && vesc_motor[i].general.info.id == id) {
      return (Motor *)&(vesc_motor[i]);
    }
  }
  return NULL;
}

void VESC_Motor_Execute(void) {
  if (VESC_Motor_Mutex == NULL) {
    VESC_Motor_Mutex = xSemaphoreCreateMutexStatic(&VESC_Motor_MutexBuffer);
  }
  xSemaphoreTake(VESC_Motor_Mutex, portMAX_DELAY);
  int32_t send_index = 0;
  uint8_t buffer[4];
  for (uint8_t i = 0; i < 8; i++) {
    if (vesc_motor[i].general.info.type == VESC_MOTOR) {
      //结构体已经初始化
      if (--vesc_motor[i].general.instruct.timeout > 0) {
        MotorInstructType alt_instruct = vesc_motor[i].general.instruct.type;
        if (alt_instruct == INSTRUCT_ALTERNATIVE) {
          alt_instruct = vesc_motor[i].general.alt_controller_update(
              (Motor *)&vesc_motor[i], vesc_motor->general.alt_controller, vesc_motor[i].general.alt_controller_param);
        }
        //指令未超时
        switch (alt_instruct) {
        case INSTRUCT_CURRENT:
          if (vesc_motor[i].general.instruct.set != 0)
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
        case INSTRUCT_ANGLE_CUMULATIVE:
          buffer_append_int32(buffer, (int32_t)(RAD2DEG(vesc_motor[i].general.instruct.set) * 1000000.0f), &send_index);
          VESC_Motor_Command(vesc_motor[i].general.info.device,
                             vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
          break;
        case INSTRUCT_ALTERNATIVE: break;
        default: break;
        }
      } else {
        //指令超时，清空指令
        vesc_motor->general.instruct.timeout = 0;
        vesc_motor->general.instruct.type    = INSTRUCT_EASE;
        vesc_motor->general.instruct.set     = 0;
        buffer_append_int32(buffer, 0, &send_index);
        VESC_Motor_Command(vesc_motor[i].general.info.device,
                           vesc_motor[i].general.info.id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
      }
    } else {
      //结构体未初始化
      //continue;
    }
  }
  xSemaphoreGive(VESC_Motor_Mutex);
}
