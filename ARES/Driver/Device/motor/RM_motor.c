/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     RM_motor.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    RM电机
  * @version  0.1
  * @date     2021-02-22
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "RM_motor.h"
#include "cmsis_os.h"
#include "semphr.h"

static RM_Motor rm_motor[2][8];

static SemaphoreHandle_t RM_Motor_Mutex = NULL;
static StaticSemaphore_t RM_Motor_MutexBuffer;

static PID_ControllerParam RM_default_speed_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = BACK_CALCULATION_INT,
                                                         .kB           = 0.015f,
                                                         .kP           = 1.5f,
                                                         .kI           = 0.1f,
                                                         .kD           = 0,
                                                         .max_Iout     = 16000.0f,
                                                         .N            = 50};
static fp32                RM_default_speed_O_Hlim      = 16384.0f;
static fp32                RM_default_speed_O_Llim      = -16384.0f;
static fp32                RM_default_speed_I_loop_Llim = 0;
static fp32                RM_default_speed_I_loop_Hlim = 0;
static ControllerConstrain RM_default_speed_constrain   = {.I_loop_Hlim = &RM_default_speed_I_loop_Hlim,
                                                         .I_loop_Llim = &RM_default_speed_I_loop_Llim,
                                                         .O_Hlim      = &RM_default_speed_O_Hlim,
                                                         .O_Llim      = &RM_default_speed_O_Llim};

static PID_ControllerParam RM_default_angle_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = BACK_CALCULATION_INT,
                                                         .kB           = 0.001f,
                                                         .kP           = 10000.0f,
                                                         .kI           = 10.0f,
                                                         .kD           = 0,
                                                         .max_Iout     = 5000.0f,
                                                         .N            = 50};
static fp32                RM_default_angle_O_Hlim      = 8000.0f;
static fp32                RM_default_angle_O_Llim      = -8000.0f;
static fp32                RM_default_angle_I_loop_Llim = 2.0f * PI;
static fp32                RM_default_angle_I_loop_Hlim = 0;
static ControllerConstrain RM_default_angle_constrain   = {.I_loop_Hlim = &RM_default_angle_I_loop_Hlim,
                                                         .I_loop_Llim = &RM_default_angle_I_loop_Llim,
                                                         .O_Hlim      = &RM_default_angle_O_Hlim,
                                                         .O_Llim      = &RM_default_angle_O_Llim};

#define MOTOR ((Motor *)self)
#define RM ((RM_Motor *)self)

#define PARSE_RM_MOTOR(ptr, RxData)                                                                                           \
  {                                                                                                                           \
    (ptr)->angle       = (fp32)((uint16_t)((RxData[0]) << 8 | (RxData[1]))) * RM_MOTOR_ECD2RAD;                               \
    (ptr)->speed       = (fp32)((int16_t)((RxData[2]) << 8 | (RxData[3])));                                                   \
    (ptr)->current     = (fp32)(int16_t)((RxData[4]) << 8 | (RxData[5]));                                                     \
    (ptr)->temperature = (RxData[6]);                                                                                         \
  }

static void RM_Motor_Init(RM_Motor *self, CAN_Device device, uint8_t id) {
  for (uint8_t i = 0; i < sizeof(RM_Motor); i++) {
    ((uint8_t *)self)[i] = 0;
  }
  MOTOR->info.device = device;
  MOTOR->info.id     = id;
  MOTOR->info.type   = RM_MOTOR;
  PID_ControllerInit(&(RM->speed_pid), &RM_default_speed_constrain, &RM_default_speed_pid_param, 0.002f);
  PID_ControllerInit(&(RM->angle_pid), &RM_default_angle_constrain, &RM_default_angle_pid_param, 0.002f);
}

void RM_Motor_RxHook(CAN_Frame *frame) {
  uint8_t id         = frame->id - 0x201;
  fp32    last_angle = rm_motor[frame->device][id].general.status.angle;
  if (rm_motor[frame->device][id].general.info.type == NONE_MOTOR ||
      xTaskGetTickCount() - rm_motor[frame->device][id].rx_timestamp > 10) {
    RM_Motor_Init(&rm_motor[frame->device][id], frame->device, id);
    PARSE_RM_MOTOR(&(rm_motor[frame->device][id].general.status), frame->data);
    rm_motor[frame->device][id].rx_timestamp = xTaskGetTickCount();
  } else if (rm_motor[frame->device][id].general.info.type == RM_MOTOR) {
    PARSE_RM_MOTOR(&(rm_motor[frame->device][id].general.status), frame->data);
    rm_motor[frame->device][id].rx_timestamp = xTaskGetTickCount();
    if (rm_motor[frame->device][id].general.status.angle - last_angle > PI) {
      rm_motor[frame->device][id].general.status.cumulative_turn--;
    } else if (rm_motor[frame->device][id].general.status.angle - last_angle < -PI) {
      rm_motor[frame->device][id].general.status.cumulative_turn++;
    }
  }
}

void RM_Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->speed_pid, (ControllerParam *)param);
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_SetAnglePID(Motor *self, PID_ControllerParam *param) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->angle_pid, (ControllerParam *)param);
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_Zero(Motor *self) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->status.zero            = MOTOR->status.angle;
  MOTOR->status.cumulative_turn = 0;
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_CURRENT;
  MOTOR->instruct.set     = mA;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_SPEED;
  MOTOR->instruct.set     = rpm;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ANGLE;
  MOTOR->instruct.set     = rad;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_SetAltController(Motor *self, Controller *alt_controller,
                               MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller)) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->alt_controller        = alt_controller;
  MOTOR->alt_controller_update = alt_controller_update;
  xSemaphoreGive(RM_Motor_Mutex);
}

void RM_Motor_AltControl(Motor *self, uint32_t timeout) {
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ALTERNATIVE;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RM_Motor_Mutex);
}

Motor *RM_Motor_Find(CAN_Device device, uint8_t id) {
  if (rm_motor[device][id].general.info.type == RM_MOTOR) {
    return (Motor *)&(rm_motor[device][id]);
  }
  return NULL;
}

static void RM_Motor_Command(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, uint16_t motor_all_id,
                             CAN_Device device) {
  uint8_t   TxData[8];
  CAN_Frame frame;
  TxData[0]    = motor1 >> 8;
  TxData[1]    = motor1;
  TxData[2]    = motor2 >> 8;
  TxData[3]    = motor2;
  TxData[4]    = motor3 >> 8;
  TxData[5]    = motor3;
  TxData[6]    = motor4 >> 8;
  TxData[7]    = motor4;
  frame.data   = TxData;
  frame.device = device;
  frame.type   = CAN_FRAME_STD;
  frame.len    = 8;
  frame.id     = motor_all_id;
  CAN_Tx(&frame);
}

void RM_Motor_Execute(void) {
  fp32 set_speed; //串级调节的中间变量
  if (RM_Motor_Mutex == NULL) {
    RM_Motor_Mutex = xSemaphoreCreateMutexStatic(&RM_Motor_MutexBuffer);
  }
  xSemaphoreTake(RM_Motor_Mutex, portMAX_DELAY);
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      if (rm_motor[i][j].general.info.type == RM_MOTOR) {
        //结构体已经初始化
        if (--rm_motor[i][j].general.instruct.timeout > 0) { //直接减法 默认1ms调用一次
          //指令未超时
          switch (rm_motor[i][j].general.instruct.type) {
          case INSTRUCT_CURRENT: rm_motor[i][j].set_current = rm_motor[i][j].general.instruct.set; break;
          case INSTRUCT_SPEED:
            rm_motor[i][j].set_current =
                controllerUpdate((Controller *)&rm_motor[i][j].speed_pid, &rm_motor[i][j].general.instruct.set,
                                 &rm_motor[i][j].general.status.speed, NULL);
            break;
          case INSTRUCT_ANGLE:
            set_speed = controllerUpdate((Controller *)&rm_motor[i][j].angle_pid, &rm_motor[i][j].general.instruct.set,
                                         &rm_motor[i][j].general.status.angle, NULL);
            rm_motor[i][j].set_current = controllerUpdate((Controller *)&rm_motor[i][j].speed_pid, &set_speed,
                                                          &rm_motor[i][j].general.status.speed, NULL);
            break;
          case INSTRUCT_ALTERNATIVE:
            rm_motor[i][j].general.alt_controller_update((Motor *)&rm_motor[i][j], rm_motor[i][j].general.alt_controller);
            break;
          default: break;
          }
        } else {
          //指令已超时
          if (rm_motor[i][j].general.instruct.timeout == 0) {
            rm_motor[i][j].general.instruct.type = INSTRUCT_CURRENT;
          } else {
            rm_motor[i][j].general.instruct.type    = INSTRUCT_EASE;
            rm_motor[i][j].general.instruct.timeout = 0;
          }
          rm_motor[i][j].set_current = rm_motor[i][j].general.instruct.set = 0;
        }
      }
    }
  }
  xSemaphoreGive(RM_Motor_Mutex);
  if (rm_motor[0][0].general.instruct.type != INSTRUCT_EASE || rm_motor[0][1].general.instruct.type != INSTRUCT_EASE ||
      rm_motor[0][2].general.instruct.type != INSTRUCT_EASE || rm_motor[0][3].general.instruct.type != INSTRUCT_EASE) {
    RM_Motor_Command(rm_motor[0][0].set_current, rm_motor[0][1].set_current, rm_motor[0][2].set_current,
                     rm_motor[0][3].set_current, RM_MOTOR_FRAME_HEAD_1, INTERNAL_CAN1);
  }
  if (rm_motor[0][4].general.instruct.type != INSTRUCT_EASE || rm_motor[0][5].general.instruct.type != INSTRUCT_EASE ||
      rm_motor[0][6].general.instruct.type != INSTRUCT_EASE || rm_motor[0][7].general.instruct.type != INSTRUCT_EASE) {
    RM_Motor_Command(rm_motor[0][4].set_current, rm_motor[0][5].set_current, rm_motor[0][6].set_current,
                     rm_motor[0][7].set_current, RM_MOTOR_FRAME_HEAD_2, INTERNAL_CAN1);
  }
  if (rm_motor[1][0].general.instruct.type != INSTRUCT_EASE || rm_motor[1][1].general.instruct.type != INSTRUCT_EASE ||
      rm_motor[1][2].general.instruct.type != INSTRUCT_EASE || rm_motor[1][3].general.instruct.type != INSTRUCT_EASE) {
    RM_Motor_Command(rm_motor[1][0].set_current, rm_motor[1][1].set_current, rm_motor[1][2].set_current,
                     rm_motor[1][3].set_current, RM_MOTOR_FRAME_HEAD_1, INTERNAL_CAN2);
  }
  if (rm_motor[1][4].general.instruct.type != INSTRUCT_EASE || rm_motor[1][5].general.instruct.type != INSTRUCT_EASE ||
      rm_motor[1][6].general.instruct.type != INSTRUCT_EASE || rm_motor[1][7].general.instruct.type != INSTRUCT_EASE) {
    RM_Motor_Command(rm_motor[1][4].set_current, rm_motor[1][5].set_current, rm_motor[1][6].set_current,
                     rm_motor[1][7].set_current, RM_MOTOR_FRAME_HEAD_2, INTERNAL_CAN2);
  }
}
