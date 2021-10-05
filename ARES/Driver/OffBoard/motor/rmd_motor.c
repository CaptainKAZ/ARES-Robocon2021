/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     RMD_motor.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    光毓RMD电机驱动程序
  * @version  0.1
  * @date     2021-03-03
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "rmd_motor.h"
#include "cmsis_os.h"
#include "semphr.h"

static RMD_Motor rmd_motor[2][4];

static SemaphoreHandle_t RMD_Motor_Mutex = NULL;
static StaticSemaphore_t RMD_Motor_MutexBuffer;

static PID_ControllerParam RMD_default_speed_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                          .Int_type     = BACK_CALCULATION_INT,
                                                          .kB           = 0.015f,
                                                          .kP           = 1.5f,
                                                          .kI           = 0.1f,
                                                          .kD           = 0,
                                                          .max_Iout     = 16000.0f,
                                                          .N            = 50};
static fp32                RMD_default_speed_O_Hlim      = 2000.0f;
static fp32                RMD_default_speed_O_Llim      = -2000.0f;
static fp32                RMD_default_speed_I_loop_Llim = 0;
static fp32                RMD_default_speed_I_loop_Hlim = 0;
static ControllerConstrain RMD_default_speed_constrain   = {.I_loop_Hlim = &RMD_default_speed_I_loop_Hlim,
                                                          .I_loop_Llim = &RMD_default_speed_I_loop_Llim,
                                                          .O_Hlim      = &RMD_default_speed_O_Hlim,
                                                          .O_Llim      = &RMD_default_speed_O_Llim};

static PID_ControllerParam RMD_default_angle_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                          .Int_type     = BACK_CALCULATION_INT,
                                                          .kB           = 0.001f,
                                                          .kP           = 10000.0f,
                                                          .kI           = 10.0f,
                                                          .kD           = 0,
                                                          .max_Iout     = 5000.0f,
                                                          .N            = 50};
static fp32                RMD_default_angle_O_Hlim      = 8000.0f;
static fp32                RMD_default_angle_O_Llim      = -8000.0f;
static fp32                RMD_default_angle_I_loop_Llim = 0;
static fp32                RMD_default_angle_I_loop_Hlim = 2 * PI;
static ControllerConstrain RMD_default_angle_constrain   = {.I_loop_Hlim = &RMD_default_angle_I_loop_Hlim,
                                                          .I_loop_Llim = &RMD_default_angle_I_loop_Llim,
                                                          .O_Hlim      = &RMD_default_angle_O_Hlim,
                                                          .O_Llim      = &RMD_default_angle_O_Llim};

#define MOTOR ((Motor *)self)
#define RM ((RMD_Motor *)self)

#define PARSE_RMD_MOTOR(ptr, RxData)                                                                                          \
  {                                                                                                                           \
    (ptr)->angle       = (fp32)((uint16_t)((RxData[7]) << 8 | (RxData[6]))) * RMD_MOTOR_ECD2RAD;                              \
    (ptr)->speed       = (fp32)((int16_t)((RxData[5]) << 8 | (RxData[4]))) * RMD_MOTOR_DPS2RPM;                               \
    (ptr)->current     = (fp32)(int16_t)((RxData[3]) << 8 | (RxData[2])) * RMD_MOTOR_I_FACTOR;                                \
    (ptr)->temperature = (RxData[1]);                                                                                         \
  }

void RMD_Motor_Init(Motor *self, CAN_Device device, uint8_t id) {
  for (uint16_t i = 0; i < sizeof(RMD_Motor); i++) {
    ((uint8_t *)self)[i] = 0;
  }
  MOTOR->info.device = device;
  MOTOR->info.id     = id;
  MOTOR->info.type   = RMD_MOTOR;
  PID_ControllerInit(&(RM->speed_pid), &RMD_default_speed_constrain, &RMD_default_speed_pid_param, 0.02f);
  PID_ControllerInit(&(RM->angle_pid), &RMD_default_angle_constrain, &RMD_default_angle_pid_param, 0.02f);
}

void RMD_Motor_RxHook(CAN_Frame *frame) {
  uint8_t id         = frame->id - 0x141;
  fp32    last_angle = rmd_motor[frame->device][id].general.status.angle;
  if (rmd_motor[frame->device][id].general.info.type == NONE_MOTOR ||
      xTaskGetTickCount() - rmd_motor[frame->device][id].rx_timestamp > 10) {
    RMD_Motor_Init((Motor *)&rmd_motor[frame->device][id], frame->device, id);
    PARSE_RMD_MOTOR(&(rmd_motor[frame->device][id].general.status), frame->data);
    rmd_motor[frame->device][id].rx_timestamp = xTaskGetTickCount();
  } else if (rmd_motor[frame->device][id].general.info.type == RMD_MOTOR) {
    PARSE_RMD_MOTOR(&(rmd_motor[frame->device][id].general.status), frame->data);
    rmd_motor[frame->device][id].rx_timestamp = xTaskGetTickCount();
    if (rmd_motor[frame->device][id].general.status.angle - last_angle > PI) {
      rmd_motor[frame->device][id].general.status.cumulativeTurn--;
    } else if (rmd_motor[frame->device][id].general.status.angle - last_angle < -PI) {
      rmd_motor[frame->device][id].general.status.cumulativeTurn++;
    }
  }
}

void RMD_Motor_SetSpeedPID(Motor *self, PID_ControllerParam *param) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->speed_pid, (ControllerParam *)param);
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_SetAnglePID(Motor *self, PID_ControllerParam *param) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->angle_pid, (ControllerParam *)param);
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_Zero(Motor *self) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->status.zero           = MOTOR->status.angle;
  MOTOR->status.cumulativeTurn = 0;
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_SetCurrent(Motor *self, fp32 mA, uint32_t timeout) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_CURRENT;
  MOTOR->instruct.set     = mA;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_SetSpeed(Motor *self, fp32 rpm, uint32_t timeout) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_SPEED;
  MOTOR->instruct.set     = rpm;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_SetAngle(Motor *self, fp32 rad, uint32_t timeout) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ANGLE;
  MOTOR->instruct.set     = rad;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_SetAltController(Motor *self, Controller *alt_controller, void *param,
                                MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller,
                                                                           void *param)) {
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->alt_controller        = alt_controller;
  MOTOR->alt_controller_update = alt_controller_update;
  MOTOR->alt_controller_param  = param;
  xSemaphoreGive(RMD_Motor_Mutex);
}

void RMD_Motor_AltControl(Motor *self, MotorInstructType type, uint32_t timeout) {
  (void)type;
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  if (MOTOR->info.type != RMD_MOTOR) {
    return;
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ALTERNATIVE;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(RMD_Motor_Mutex);
}

Motor *RMD_Motor_Find(CAN_Device device, uint8_t id) {
  if (rmd_motor[device][id].general.info.type == RMD_MOTOR) {
    return (Motor *)&(rmd_motor[device][id]);
  }
  return NULL;
}

static void RMD_Motor_Command(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, uint16_t motor_all_id,
                              CAN_Device device) {
  uint8_t   TxData[8];
  CAN_Frame frame;
  TxData[0]    = motor1;
  TxData[1]    = motor1 >> 8;
  TxData[2]    = motor2;
  TxData[3]    = motor2 >> 8;
  TxData[4]    = motor3;
  TxData[5]    = motor3 >> 8;
  TxData[6]    = motor4;
  TxData[7]    = motor4 >> 8;
  frame.data   = TxData;
  frame.device = device;
  frame.type   = CAN_FRAME_STD;
  frame.len    = 8;
  frame.id     = motor_all_id;
  CAN_Tx(&frame);
}

void RMD_Motor_Execute(void) {
  fp32 set_speed; //串级调节的中间变量
  if (RMD_Motor_Mutex == NULL) {
    RMD_Motor_Mutex = xSemaphoreCreateMutexStatic(&RMD_Motor_MutexBuffer);
  }
  xSemaphoreTake(RMD_Motor_Mutex, portMAX_DELAY);
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      if (rmd_motor[i][j].general.info.type == RMD_MOTOR) {
        //结构体已经初始化
        if (--rmd_motor[i][j].general.instruct.timeout > 0) { //直接减法 默认1ms调用一次
          //指令未超时
          switch (rmd_motor[i][j].general.instruct.type) {
          case INSTRUCT_CURRENT: rmd_motor[i][j].set_current = rmd_motor[i][j].general.instruct.set; break;
          case INSTRUCT_SPEED:
            rmd_motor[i][j].set_current =
                controllerUpdate((Controller *)&rmd_motor[i][j].speed_pid, &rmd_motor[i][j].general.instruct.set,
                                 &rmd_motor[i][j].general.status.speed, NULL);
            break;
          case INSTRUCT_ANGLE:
            set_speed = controllerUpdate((Controller *)&rmd_motor[i][j].angle_pid, &rmd_motor[i][j].general.instruct.set,
                                         &rmd_motor[i][j].general.status.angle, NULL);
            rmd_motor[i][j].set_current = controllerUpdate((Controller *)&rmd_motor[i][j].speed_pid, &set_speed,
                                                           &rmd_motor[i][j].general.status.speed, NULL);
            break;
          case INSTRUCT_ALTERNATIVE:
            rmd_motor[i][j].general.alt_controller_update((Motor *)&rmd_motor[i][j], rmd_motor[i][j].general.alt_controller,
                                                          rmd_motor[i][j].general.alt_controller_param);
            break;
          default: break;
          }
        } else {
          //指令已超时
          if (rmd_motor[i][j].general.instruct.timeout == 0) {
            rmd_motor[i][j].general.instruct.type = INSTRUCT_CURRENT;
          } else {
            rmd_motor[i][j].general.instruct.type    = INSTRUCT_EASE;
            rmd_motor[i][j].general.instruct.timeout = 0;
          }
          rmd_motor[i][j].set_current = rmd_motor[i][j].general.instruct.set = 0;
        }
      }
    }
  }
  xSemaphoreGive(RMD_Motor_Mutex);

  RMD_Motor_Command(rmd_motor[0][0].set_current, rmd_motor[0][1].set_current, rmd_motor[0][2].set_current,
                    rmd_motor[0][3].set_current, RMD_MULTI_FRAME_HEAD, INTERNAL_CAN1);
  RMD_Motor_Command(rmd_motor[1][0].set_current, rmd_motor[1][1].set_current, rmd_motor[1][2].set_current,
                    rmd_motor[1][3].set_current, RMD_MULTI_FRAME_HEAD, INTERNAL_CAN2);
}
