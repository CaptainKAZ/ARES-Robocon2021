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
#include "rm_motor.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "monitor_task.h"

MonitorList rmMotorMonitor;

static RM_Motor rmMotor[4][8];

static SemaphoreHandle_t rmMotorMutex = NULL;
static StaticSemaphore_t rmMotorMutexBuffer;

static PID_ControllerParam RM_default_speed_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = BACK_CALCULATION_INT,
                                                         .kB           = 0.015f,
                                                         .kP           = 1.5f,
                                                         .kI           = 0.1f,
                                                         .kD           = 0,
                                                         .max_Iout     = 16000.0f,
                                                         .N            = 50};
static fp32                RM_default_speed_O_Hlim      = 10000.0f;
static fp32                RM_default_speed_O_Llim      = -10000.0f;
static fp32                RM_default_speed_I_loop_Llim = 0;
static fp32                RM_default_speed_I_loop_Hlim = 0;
static ControllerConstrain RM_default_speed_constrain   = {.I_loop_Hlim = &RM_default_speed_I_loop_Hlim,
                                                         .I_loop_Llim = &RM_default_speed_I_loop_Llim,
                                                         .O_Hlim      = &RM_default_speed_O_Hlim,
                                                         .O_Llim      = &RM_default_speed_O_Llim};
static fp32                M3508_speed_O_Hlim           = 16384.0f;
static fp32                M3508_speed_O_Llim           = -16384.0f;
static fp32                M3508_speed_I_loop_Llim      = 0;
static fp32                M3508_speed_I_loop_Hlim      = 0;
static ControllerConstrain M3508_speed_constrain        = {.I_loop_Hlim = &M3508_speed_I_loop_Hlim,
                                                    .I_loop_Llim = &M3508_speed_I_loop_Llim,
                                                    .O_Hlim      = &M3508_speed_O_Hlim,
                                                    .O_Llim      = &M3508_speed_O_Llim};
static fp32                M2006_speed_O_Hlim           = 10000.0f;
static fp32                M2006_speed_O_Llim           = -10000.0f;
static fp32                M2006_speed_I_loop_Llim      = 0;
static fp32                M2006_speed_I_loop_Hlim      = 0;
static ControllerConstrain M2006_speed_constrain        = {.I_loop_Hlim = &M2006_speed_I_loop_Hlim,
                                                    .I_loop_Llim = &M2006_speed_I_loop_Llim,
                                                    .O_Hlim      = &M2006_speed_O_Hlim,
                                                    .O_Llim      = &M2006_speed_O_Llim};
static fp32                GM6020_speed_O_Hlim          = 30000.0f;
static fp32                GM6020_speed_O_Llim          = -30000.0f;
static fp32                GM6020_speed_I_loop_Llim     = 0;
static fp32                GM6020_speed_I_loop_Hlim     = 0;
static ControllerConstrain GM6020_speed_constrain       = {.I_loop_Hlim = &GM6020_speed_I_loop_Hlim,
                                                     .I_loop_Llim = &GM6020_speed_I_loop_Llim,
                                                     .O_Hlim      = &GM6020_speed_O_Hlim,
                                                     .O_Llim      = &GM6020_speed_O_Llim};

static PID_ControllerParam RM_default_angle_pid_param   = {.general.type = PIDPOS_CONTROLLER,
                                                         .Int_type     = BACK_CALCULATION_INT,
                                                         .kB           = 0.001f,
                                                         .kP           = 10000.0f,
                                                         .kI           = 10.0f,
                                                         .kD           = 0,
                                                         .max_Iout     = 5000.0f,
                                                         .N            = 50};
static fp32                RM_default_angle_O_Hlim      = 100.0f;
static fp32                RM_default_angle_O_Llim      = -100.0f;
static fp32                RM_default_angle_I_loop_Llim = 0;
static fp32                RM_default_angle_I_loop_Hlim = 2.0f * PI;
static ControllerConstrain RM_default_angle_constrain   = {.I_loop_Hlim = &RM_default_angle_I_loop_Hlim,
                                                         .I_loop_Llim = &RM_default_angle_I_loop_Llim,
                                                         .O_Hlim      = &RM_default_angle_O_Hlim,
                                                         .O_Llim      = &RM_default_angle_O_Llim};
static fp32                M3508_angle_O_Hlim           = 486.0f;
static fp32                M3508_angle_O_Llim           = -486.0f;
static ControllerConstrain M3508_angle_constrain        = {.I_loop_Hlim = &RM_default_angle_I_loop_Hlim,
                                                    .I_loop_Llim = &RM_default_angle_I_loop_Llim,
                                                    .O_Hlim      = &M3508_angle_O_Hlim,
                                                    .O_Llim      = &M3508_angle_O_Llim};
static fp32                M2006_angle_O_Hlim           = 500.0f;
static fp32                M2006_angle_O_Llim           = -500.0f;
static ControllerConstrain M2006_angle_constrain        = {.I_loop_Hlim = &RM_default_I_loop_Hlim,
                                                    .I_loop_Llim = &RM_default_I_loop_Llim,
                                                    .O_Hlim      = &M2006_angle_O_Hlim,
                                                    .O_Llim      = &M2006_angle_O_Llim};
static fp32                GM6020_angle_O_Hlim          = 320.0f;
static fp32                GM6020_angle_O_Llim          = -320.0f;
static ControllerConstrain GM6020_angle_constrain       = {.I_loop_Hlim = &RM_default_angle_I_loop_Hlim,
                                                     .I_loop_Llim = &RM_default_angle_I_loop_Llim,
                                                     .O_Hlim      = &GM6020_angle_O_Hlim,
                                                     .O_Llim      = &GM6020_angle_O_Llim};

#define MOTOR ((Motor *)self)
#define RM ((RM_Motor *)self)

#define PARSE_RM_MOTOR(ptr, RxData)                                                                                           \
  {                                                                                                                           \
    (ptr)->angle       = (fp32)((uint16_t)((RxData[0]) << 8 | (RxData[1]))) * RM_MOTOR_ECD2RAD - (ptr)->zero;                 \
    (ptr)->speed       = (fp32)((int16_t)((RxData[2]) << 8 | (RxData[3])));                                                   \
    (ptr)->current     = (fp32)(int16_t)((RxData[4]) << 8 | (RxData[5]));                                                     \
    (ptr)->temperature = (RxData[6]);                                                                                         \
  }

static void RM_Motor_init(RM_Motor *self, CAN_Device device, uint8_t id) {
  for (uint8_t i = 0; i < sizeof(RM_Motor); i++) {
    ((uint8_t *)self)[i] = 0;
  }
  MOTOR->reductionRatio = 1;
  MOTOR->info.device    = device;
  MOTOR->info.id        = id;
  MOTOR->info.type      = RM_MOTOR;
  PID_ControllerInit(&(RM->speed_pid), &RM_default_speed_constrain, &RM_default_speed_pid_param, 0.002f);
  PID_ControllerInit(&(RM->angle_pid), &RM_default_angle_constrain, &RM_default_angle_pid_param, 0.002f);
}

__inline static void RM_Motor_reduce(Motor *motor) {
  if (motor->reductionRatio != 0) {
    motor->status.speedOutput = motor->status.speed / motor->reductionRatio;
    motor->status.angleOutput = (motor->status.angle + 2 * PI * motor->status.cumulativeTurn) / motor->reductionRatio;
  } else {
    motor->status.angleOutput = motor->status.angle;
    motor->status.speedOutput = motor->status.speed;
  }
}

void RM_Motor_rxHook(CAN_Frame *frame) {
  uint8_t id         = frame->id - 0x201;
  fp32    last_angle = rmMotor[frame->device][id].general.status.angle;
  if (rmMotor[frame->device][id].general.info.type == NONE_MOTOR) {
    RM_Motor_init(&rmMotor[frame->device][id], frame->device, id);
    PARSE_RM_MOTOR(&(rmMotor[frame->device][id].general.status), frame->data);
    rmMotor[frame->device][id].rx_timestamp = xTaskGetTickCount();
    RM_Motor_reduce((Motor *)&(rmMotor[frame->device][id]));
    return;
  }

  PARSE_RM_MOTOR(&(rmMotor[frame->device][id].general.status), frame->data);
  rmMotor[frame->device][id].rx_timestamp = xTaskGetTickCount();
  RM_Motor_reduce((Motor *)&(rmMotor[frame->device][id]));

  if (rmMotor[frame->device][id].general.status.angle - last_angle > PI) {
    rmMotor[frame->device][id].general.status.cumulativeTurn--;
  } else if (rmMotor[frame->device][id].general.status.angle - last_angle < -PI) {
    rmMotor[frame->device][id].general.status.cumulativeTurn++;
  }
}

void RM_Motor_setSpeedPid(Motor *self, PID_ControllerParam *param) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->speed_pid, (ControllerParam *)param);
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setAnglePid(Motor *self, PID_ControllerParam *param) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  controllerSetParam((Controller *)&RM->angle_pid, (ControllerParam *)param);
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_zero(Motor *self) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->status.zero           = MOTOR->status.angle;
  MOTOR->status.angle          = 0;
  MOTOR->status.cumulativeTurn = 0;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setCurrent(Motor *self, fp32 mA, uint32_t timeout) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_CURRENT;
  MOTOR->instruct.set     = mA;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setSpeed(Motor *self, fp32 rpm, uint32_t timeout) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_SPEED;
  MOTOR->instruct.set     = rpm;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setAngle(Motor *self, fp32 rad, uint32_t timeout) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ANGLE;
  MOTOR->instruct.set     = rad;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setAngleCumulative(Motor *self, fp32 rad, uint32_t timeout) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ANGLE_CUMULATIVE;
  MOTOR->instruct.set     = rad;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_setAltController(Motor *self, Controller *alt_controller, void *param,
                               MotorInstructType (*alt_controller_update)(Motor *motor, Controller *controller, void *param)) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->alt_controller        = alt_controller;
  MOTOR->alt_controller_update = alt_controller_update;
  MOTOR->alt_controller_param  = param;
  xSemaphoreGive(rmMotorMutex);
}

void RM_Motor_altControl(Motor *self, uint32_t timeout) {
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  if (MOTOR->info.type != RM_MOTOR) {
    return;
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  MOTOR->instruct.type    = INSTRUCT_ALTERNATIVE;
  MOTOR->instruct.timeout = timeout;
  xSemaphoreGive(rmMotorMutex);
}

Motor *RM_Motor_get(CAN_Device device, uint8_t id) {
  if (rmMotor[device][id].general.info.type == RM_MOTOR) {
    return (Motor *)&(rmMotor[device][id]);
  }
  return NULL;
}

static void RM_Motor_command(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, uint16_t motor_all_id,
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

void RM_Motor_execute(void) {
  fp32  set_speed; //串级调节的中间变量
  fp32 *Hlim;
  if (rmMotorMutex == NULL) {
    rmMotorMutex = xSemaphoreCreateMutexStatic(&rmMotorMutexBuffer);
  }
  xSemaphoreTake(rmMotorMutex, portMAX_DELAY);
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      if (rmMotor[i][j].general.info.type != RM_MOTOR) {
        continue;
      }
      //结构体已经初始化
      if (--rmMotor[i][j].general.instruct.timeout > 0) { //直接减法 默认1ms调用一次
        //指令未超时
        MotorInstructType instructType = rmMotor[i][j].general.instruct.type;
        if (instructType == INSTRUCT_ALTERNATIVE) {
          if (rmMotor[i][j].general.alt_controller_update) {
            instructType = rmMotor[i][j].general.alt_controller_update(
                (Motor *)&rmMotor[i][j], rmMotor[i][j].general.alt_controller, rmMotor[i][j].general.alt_controller_param);
          } else {
            rmMotor[i][j].set_current = 0;
          }
        }

        switch (instructType) {
        case INSTRUCT_CURRENT: rmMotor[i][j].set_current = rmMotor[i][j].general.instruct.set; break;
        case INSTRUCT_SPEED:
          rmMotor[i][j].set_current =
              controllerUpdate((Controller *)&rmMotor[i][j].speed_pid, &rmMotor[i][j].general.instruct.set,
                               &rmMotor[i][j].general.status.speedOutput, NULL);
          break;
        case INSTRUCT_ANGLE:
          set_speed = controllerUpdate((Controller *)&rmMotor[i][j].angle_pid, &rmMotor[i][j].general.instruct.set,
                                       &rmMotor[i][j].general.status.angleOutput, NULL);
          rmMotor[i][j].set_current = controllerUpdate((Controller *)&rmMotor[i][j].speed_pid, &set_speed,
                                                       &rmMotor[i][j].general.status.speedOutput, NULL);
          break;
        case INSTRUCT_ANGLE_CUMULATIVE:
          Hlim                                                   = rmMotor[i][j].angle_pid.general.constrain->I_loop_Hlim;
          rmMotor[i][j].angle_pid.general.constrain->I_loop_Hlim = rmMotor[i][j].angle_pid.general.constrain->I_loop_Llim;
          set_speed = controllerUpdate((Controller *)&rmMotor[i][j].angle_pid, &rmMotor[i][j].general.instruct.set,
                                       &rmMotor[i][j].general.status.angleOutput, NULL);
          rmMotor[i][j].set_current = controllerUpdate((Controller *)&rmMotor[i][j].speed_pid, &set_speed,
                                                       &rmMotor[i][j].general.status.speedOutput, NULL);
          rmMotor[i][j].angle_pid.general.constrain->I_loop_Hlim = Hlim;
          break;
        case INSTRUCT_ALTERNATIVE: break;
        default: break;
        }
      } else {
        //指令已超时
        if (rmMotor[i][j].general.instruct.timeout == 0) {
          rmMotor[i][j].general.instruct.type = INSTRUCT_CURRENT;
        } else {
          rmMotor[i][j].general.instruct.type    = INSTRUCT_EASE;
          rmMotor[i][j].general.instruct.timeout = 0;
        }
        rmMotor[i][j].set_current = rmMotor[i][j].general.instruct.set = 0;
      }
    }
  }
  xSemaphoreGive(rmMotorMutex);
  for (uint8_t i = 0; i < 4; i++) {
    if (rmMotor[i][0].general.instruct.type != INSTRUCT_EASE || rmMotor[i][1].general.instruct.type != INSTRUCT_EASE ||
        rmMotor[i][2].general.instruct.type != INSTRUCT_EASE || rmMotor[i][3].general.instruct.type != INSTRUCT_EASE) {
      RM_Motor_command(rmMotor[i][0].set_current, rmMotor[i][1].set_current, rmMotor[i][2].set_current,
                       rmMotor[i][3].set_current, RM_MOTOR_FRAME_HEAD_1, INTERNAL_CAN1);
    }
  }
}

void RM_Motor_setAsM3508(Motor *self) {
  self->reductionRatio            = M3508_REDUCTION_RATIO;
  RM->angle_pid.general.constrain = &M3508_angle_constrain;
  RM->speed_pid.general.constrain = &M3508_speed_constrain;
}

void RM_Motor_setAsM2006(Motor *self) {
  self->reductionRatio            = M2006_REDUCTION_RATIO;
  RM->angle_pid.general.constrain = &M2006_angle_constrain;
  RM->speed_pid.general.constrain = &M2006_speed_constrain;
}

void RM_Motor_setAsGm6020(Motor *self) {
  RM->angle_pid.general.constrain = &GM6020_angle_constrain;
  RM->speed_pid.general.constrain = &GM6020_speed_constrain;
}
