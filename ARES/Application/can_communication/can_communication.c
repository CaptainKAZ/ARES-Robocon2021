/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  * 
  * @file     can_communication.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note     通过固定速率RTOS任务计算PID并且完成发送
  * @version  1.0
  * @date     2020-10-12
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */

#include "can_communication.h"

#include "tim.h"
#include "can.h"
#include <stdlib.h>
#include <string.h>

// RM底盘电机数据读取
#define rm_get_motor_status(ptr, RxData)                                                                                      \
  {                                                                                                                           \
    (ptr)->last_angle       = (ptr)->angle;                                                                                   \
    (ptr)->angle            = (fp32)((uint16_t)((RxData[0]) << 8 | (RxData[1]))) * RM_ECD2RAD;                                \
    (ptr)->rpm              = (fp32)((int16_t)((RxData[2]) << 8 | (RxData[3])));                                              \
    (ptr)->current          = (fp32)(int16_t)((RxData[4]) << 8 | (RxData[5]));                                                \
    (ptr)->temperature      = (RxData[6]);                                                                                    \
    (ptr)->angle_time_stamp = xTaskGetTickCount();                                                                            \
    (ptr)->rpm_time_stamp   = (ptr)->angle_time_stamp;                                                                        \
  }

//统一CAN接收函数
static void MOTOR_CAN_HOOK(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxMsgHdr, uint8_t *RxData);

//CAN帧头
CAN_TxHeaderTypeDef TxMsgHdr;
CAN_RxHeaderTypeDef RxMsgHdr;

//声明统一电机结构体组
static motor_t    motor[32];
static const fp32 default_pid[3] = {RM_DEFAULT_KP, RM_DEFAULT_KI, RM_DEFAULT_KD};
static PidTypeDef motor_PID[32];
static int8_t     motor_PID_used = 0;

// can扩展帧发送函数
static void CAN_TRANSMIT_EID(uint32_t id, uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan);
//缓冲区处理函数
__forceinline void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}
__forceinline int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res = ((uint32_t)buffer[*index]) << 24 | ((uint32_t)buffer[*index + 1]) << 16 | ((uint32_t)buffer[*index + 2]) << 8 |
                ((uint32_t)buffer[*index + 3]);
  *index += 4;
  return res;
}
__forceinline int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res = ((uint16_t)buffer[*index]) << 8 | ((uint16_t)buffer[*index + 1]);
  *index += 2;
  return res;
}
//以电流环方式控制底盘电机
static void VESC_CAN_CMD_CURRENT(uint8_t id, int32_t current, CAN_HandleTypeDef *hcan);
//以速度环方式控制底盘电机
static void VESC_CAN_CMD_RPM(uint8_t id, int32_t RPM, CAN_HandleTypeDef *hcan);
//以位置环方式控制底盘电机
static void VESC_CAN_CMD_ANGLE(uint8_t id, fp32 position, CAN_HandleTypeDef *hcan);
//以占空比方式控制底盘电机
static void VESC_CAN_CMD_DUTY(uint8_t id, fp32 duty, CAN_HandleTypeDef *hcan);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_StatusTypeDef HAL_Status;
  uint8_t           CAN_RxData[8];
  HAL_Status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsgHdr, CAN_RxData);
  if (HAL_OK == HAL_Status) {
    MOTOR_CAN_HOOK(hcan, &RxMsgHdr, CAN_RxData);
  }
}

//统一CAN接收函数
void MOTOR_CAN_HOOK(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxMsgHdr, uint8_t *RxData) {
  static uint8_t id = 0;
  if (RxMsgHdr->IDE == CAN_ID_STD) {
    //标准帧 解析成RM电机
    switch (RxMsgHdr->StdId) {
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
      if (hcan->Instance == CAN1) {
        id = RxMsgHdr->StdId - 0x201;
      } else {
        id = (RxMsgHdr->StdId - 0x201) + 8;
      }

      if (motor[id].motor_type == NONE) {
        //电机对象未初始化
        motor[id].motor_type                      = RM_MOTOR;
        motor[id].motor_instruct.transmitted_flag = 0;
        motor[id].motor_instruct.instruct_type    = CURRENT;
        motor[id].motor_instruct.set_current      = 0;
        motor[id].motor_instruct.motor_speed_pid  = &(motor_PID[motor_PID_used++]);
        PID_Init(motor[id].motor_instruct.motor_speed_pid, PID_POSITION, default_pid, RM_CURRENT_PID_MAX_OUT,
                 RM_DEFAULT_MAX_IOUT);
        motor[id].motor_instruct.motor_angle_pid = &(motor_PID[motor_PID_used++]);
        PID_Init(motor[id].motor_instruct.motor_angle_pid, PID_POSITION_RAD_LOOP, default_pid, RM_RPM_PID_MAX_OUT,
                 RM_DEFAULT_MAX_IOUT);
        motor[id].motor_can                        = hcan;
        motor[id].motor_status.cumulative_position = 0.0f;
        motor[id].motor_status.cumulative_turn     = 0;
      }
      //电机对象已初始化
      rm_get_motor_status(&motor[id].motor_status, RxData);
      if (motor[id].motor_status.angle - motor[id].motor_status.last_angle > PI) {
        //反转一圈
        motor[id].motor_status.cumulative_turn--;
      } else if (motor[id].motor_status.angle - motor[id].motor_status.last_angle < -PI) {
        //正转一圈
        motor[id].motor_status.cumulative_turn++;
      }
      motor[id].motor_status.cumulative_position =
          motor[id].motor_status.cumulative_turn * 2 * PI + motor[id].motor_status.angle;

      // TODO:detect_task hook
      break;
    default: //不合法帧
      break;
    }
  } else {
    //扩展帧 解析成VESC
    id = ((RxMsgHdr->ExtId) & 0x03) + 16;
    if (id < 32) { //检测id合法性
      if (motor[id].motor_type == NONE) {
        //电机对象未初始化
        motor[id].motor_type                       = VESC_MOTOR;
        motor[id].motor_instruct.instruct_type     = CURRENT;
        motor[id].motor_instruct.motor_speed_pid   = NULL;
        motor[id].motor_instruct.set_current       = 0;
        motor[id].motor_instruct.transmitted_flag  = 0;
        motor[id].motor_can                        = hcan;
        motor[id].motor_status.cumulative_position = 0;
      }
      //电机对象已初始化
      static int32_t ind = 0;
      switch ((RxMsgHdr->ExtId) >> 8) {
      case CAN_PACKET_STATUS:
        ind                                   = 0;
        motor[id].motor_status.rpm_time_stamp = xTaskGetTickCount();
        motor[id].motor_status.rpm            = buffer_get_int32(RxData, &ind);
        motor[id].motor_status.current        = (fp32)buffer_get_int16(RxData, &ind) / 10.0f;
        break;
      case CAN_PACKET_STATUS_2:
      case CAN_PACKET_STATUS_3:
      case CAN_PACKET_STATUS_4:
        ind                                     = 0;
        motor[id].motor_status.angle_time_stamp = xTaskGetTickCount();
        motor[id].motor_status.last_angle       = motor[id].motor_status.angle;
        ind += 2; //跳过电调温度
        motor[id].motor_status.temperature = (uint8_t)buffer_get_int16(RxData, &ind) / 10.0f;
        motor[id].motor_status.angle       = (fp32)buffer_get_int16(RxData, &ind) / 50.0f;
        if (motor[id].motor_status.angle - motor[id].motor_status.last_angle > PI) {
          //反转一圈
          motor[id].motor_status.cumulative_turn--;
        } else if (motor[id].motor_status.angle - motor[id].motor_status.last_angle < -PI) {
          //正转一圈
          motor[id].motor_status.cumulative_turn++;
        }
        motor[id].motor_status.cumulative_position =
            motor[id].motor_status.cumulative_turn * 2.0f * PI + motor[id].motor_status.angle;
        break;
      default:
        break;
        // TODO:detect_task hook
      }
    }
  }
}

//统一电机更新函数
void motor_set_speed_pid(uint8_t id, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout) {
  while (motor[id].motor_instruct.motor_speed_pid == NULL)
    vTaskDelay(20);
  xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
  {
    PID_clear(motor[id].motor_instruct.motor_speed_pid);
    motor[id].motor_instruct.motor_speed_pid->Kp       = kp;
    motor[id].motor_instruct.motor_speed_pid->Ki       = ki;
    motor[id].motor_instruct.motor_speed_pid->Kd       = kd;
    motor[id].motor_instruct.motor_speed_pid->max_iout = max_iout;
    motor[id].motor_instruct.motor_speed_pid->max_out  = max_out;
  }
  xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
}

void motor_set_angle_pid(uint8_t id, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout) {
  while (motor[id].motor_instruct.motor_angle_pid == NULL)
    vTaskDelay(20);
  xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
  {
    PID_clear(motor[id].motor_instruct.motor_angle_pid);
    motor[id].motor_instruct.motor_angle_pid->Kp       = kp;
    motor[id].motor_instruct.motor_angle_pid->Ki       = ki;
    motor[id].motor_instruct.motor_angle_pid->Kd       = kd;
    motor[id].motor_instruct.motor_angle_pid->max_out  = max_out;
    motor[id].motor_instruct.motor_angle_pid->max_iout = max_iout;
  }
  xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
}

void motor_set_rpm(uint8_t id, fp32 rpm) {
  if (motor[id].motor_type != NONE) {
    xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
    {
      if (motor[id].motor_instruct.instruct_type != RPM) {
        motor[id].motor_instruct.instruct_type = RPM;
        PID_clear(motor[id].motor_instruct.motor_speed_pid);
      }
      motor[id].motor_instruct.instruct_type    = RPM;
      motor[id].motor_instruct.set_rpm          = rpm;
      motor[id].motor_instruct.transmitted_flag = 0;
    }
    xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
  }
}

void motor_set_angle(uint8_t id, fp32 angle) {
  if (motor[id].motor_type != NONE) {
    xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
    {
      if (motor[id].motor_instruct.instruct_type != ANGLE) {
        motor[id].motor_instruct.instruct_type = ANGLE;
        PID_clear(motor[id].motor_instruct.motor_angle_pid);
        PID_clear(motor[id].motor_instruct.motor_speed_pid);
      }
      motor[id].motor_instruct.set_angle        = angle;
      motor[id].motor_instruct.transmitted_flag = 0;
    }
    xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
  }
}

void motor_set_duty(uint8_t id, fp32 duty) {
  if (motor[id].motor_type != NONE) {
    xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
    {
      motor[id].motor_instruct.instruct_type    = DUTY;
      motor[id].motor_instruct.set_duty         = duty;
      motor[id].motor_instruct.transmitted_flag = 0;
    }
    xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
  }
}

void motor_set_current(uint8_t id, fp32 current) {
  xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
  {
    motor[id].motor_instruct.instruct_type    = CURRENT;
    motor[id].motor_instruct.set_current      = current;
    motor[id].motor_instruct.transmitted_flag = 0;
  }
  xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
}

// RM底盘电机指令发送
void RM_CAN_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, uint16_t motor_all_id,
                CAN_HandleTypeDef *hcan) {
  uint8_t TxData[8];
  TxMsgHdr.StdId = motor_all_id;
  TxMsgHdr.IDE   = CAN_ID_STD;
  TxMsgHdr.RTR   = CAN_RTR_DATA;
  TxMsgHdr.DLC   = 0x08;
  TxData[0]      = motor1 >> 8;
  TxData[1]      = motor1;
  TxData[2]      = motor2 >> 8;
  TxData[3]      = motor2;
  TxData[4]      = motor3 >> 8;
  TxData[5]      = motor3;
  TxData[6]      = motor4 >> 8;
  TxData[7]      = motor4;
  HAL_CAN_AddTxMessage(hcan, &TxMsgHdr, TxData, NULL);
}

inline static void CAN_TRANSMIT_EID(uint32_t id, uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan) {
  if (len > 8) {
    len = 8;
  }

  TxMsgHdr.IDE   = CAN_ID_EXT; // extrenal frame
  TxMsgHdr.ExtId = id;
  TxMsgHdr.RTR   = CAN_RTR_DATA; // normal data
  TxMsgHdr.DLC   = len;

  HAL_CAN_AddTxMessage(hcan, &TxMsgHdr, data, NULL);
}

inline static void VESC_CAN_CMD_CURRENT(uint8_t id, int32_t current, CAN_HandleTypeDef *hcan) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, current, &send_index);
  CAN_TRANSMIT_EID(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, hcan);
}

inline static void VESC_CAN_CMD_RPM(uint8_t id, int32_t RPM, CAN_HandleTypeDef *hcan) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, RPM, &send_index);

  CAN_TRANSMIT_EID(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index, hcan);
}

inline static void VESC_CAN_CMD_ANGLE(uint8_t id, fp32 position, CAN_HandleTypeDef *hcan) {

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(position * 1000000.0f), &send_index);
  CAN_TRANSMIT_EID(id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index, hcan);
}

inline static void VESC_CAN_CMD_DUTY(uint8_t id, fp32 duty, CAN_HandleTypeDef *hcan) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
  CAN_TRANSMIT_EID(id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index, hcan);
}

void RM_CAN_CMD_RESET_ID(CAN_HandleTypeDef *hcan) {
  uint8_t TxData[8];
  TxMsgHdr.StdId = 0x700;
  TxMsgHdr.IDE   = CAN_ID_STD;
  TxMsgHdr.RTR   = CAN_RTR_DATA;
  TxMsgHdr.DLC   = 0x08;
  TxData[0]      = 0;
  TxData[1]      = 0;
  TxData[2]      = 0;
  TxData[3]      = 0;
  TxData[4]      = 0;
  TxData[5]      = 0;
  TxData[6]      = 0;
  TxData[7]      = 0;
  HAL_CAN_AddTxMessage(hcan, &TxMsgHdr, TxData, NULL);
}

static void RM_process_motor(uint8_t group) {
  for (uint8_t i = group * 4 - 4; i < group * 4; i++) {
    if (motor[i].motor_type == NONE) {
      motor[i].motor_instruct.set_current = 0;
    } else if (motor[i].motor_type == RM_MOTOR) {
      xSemaphoreTake(motor[i].motor_instruct.instruct_mutex, portMAX_DELAY);
      {
        switch (motor[i].motor_instruct.instruct_type) {
        case ANGLE:
          motor[i].motor_instruct.set_rpm = PID_Calc(motor[i].motor_instruct.motor_angle_pid, motor[i].motor_status.angle,
                                                     motor[i].motor_instruct.set_angle);
          motor[i].motor_instruct.set_current =
              PID_Calc(motor[i].motor_instruct.motor_speed_pid, motor[i].motor_status.rpm, motor[i].motor_instruct.set_rpm);
          break;
        case RPM:
          motor[i].motor_instruct.set_current =
              PID_Calc(motor[i].motor_instruct.motor_speed_pid, motor[i].motor_status.rpm, motor[i].motor_instruct.set_rpm);
          break;
        case DUTY: motor[i].motor_instruct.set_current = motor[i].motor_instruct.set_duty * RM_CURRENT_MAX_OUT; break;
        case CURRENT: break;
        default: motor[i].motor_instruct.set_current = 0; break;
        }
        //motor[i].motor_instruct.transmitted_flag = 1;
      }
      xSemaphoreGive(motor[i].motor_instruct.instruct_mutex);
    }
  }
}

void instruct_motor(uint8_t id) {
  if (motor[id].motor_type != NONE && motor[id].motor_instruct.transmitted_flag == 0) {
    if (id < 4) {
      RM_process_motor(1);
      RM_CAN_CMD(motor[0].motor_instruct.set_current, motor[1].motor_instruct.set_current, motor[2].motor_instruct.set_current,
                 motor[3].motor_instruct.set_current, 0x200, &hcan1);
    } else if (id < 8) {
      RM_process_motor(2);
      RM_CAN_CMD(motor[4].motor_instruct.set_current, motor[5].motor_instruct.set_current, motor[6].motor_instruct.set_current,
                 motor[7].motor_instruct.set_current, 0x1FF, &hcan1);

    } else if (id < 12) {
      RM_process_motor(3);
      RM_CAN_CMD(motor[8].motor_instruct.set_current, motor[9].motor_instruct.set_current,
                 motor[10].motor_instruct.set_current, motor[11].motor_instruct.set_current, 0x200, &hcan2);

    } else if (id < 16) {
      RM_process_motor(4);
      RM_CAN_CMD(motor[12].motor_instruct.set_current, motor[13].motor_instruct.set_current,
                 motor[14].motor_instruct.set_current, motor[15].motor_instruct.set_current, 0x1FF, &hcan2);
    } else if (id < 32) {
      xSemaphoreTake(motor[id].motor_instruct.instruct_mutex, portMAX_DELAY);
      {
        if (motor[id].motor_instruct.instruct_type == ANGLE) {
          VESC_CAN_CMD_ANGLE(id - 16, motor[id].motor_instruct.set_angle * 50, motor[id].motor_can);
          motor[id].motor_instruct.transmitted_flag = 1;
        } else if (motor[id].motor_instruct.instruct_type == RPM) {
          VESC_CAN_CMD_RPM(id - 16, motor[id].motor_instruct.set_rpm, motor[id].motor_can);
        } else if (motor[id].motor_instruct.instruct_type == DUTY) {
          VESC_CAN_CMD_DUTY(id - 16, motor[id].motor_instruct.set_duty, motor[id].motor_can);
        } else if (motor[id].motor_instruct.instruct_type == CURRENT) {
          VESC_CAN_CMD_CURRENT(id - 16, motor[id].motor_instruct.set_current, motor[id].motor_can);
        }
      }
      xSemaphoreGive(motor[id].motor_instruct.instruct_mutex);
    }
  }
}

const motor_t *get_motor_id_pointer(uint8_t id) { return &motor[id]; }
const motor_t *get_motor_all_pointer() { return &motor[0]; };

void motor_clear_cumulative_turn(uint8_t id) { motor[id].motor_status.cumulative_turn = 0; }

void can_communication_task(void *pvParameters) {
  TickType_t xLastWakeTime;
  CAN_Start(&hcan1);
  CAN_Start(&hcan2);

  for (uint8_t i = 0; i < 32; i++) {
    motor[i].motor_instruct.instruct_mutex = xSemaphoreCreateMutex();
    while (motor[i].motor_instruct.instruct_mutex == NULL)
      ;
  }
  vTaskDelay(MOTOR_INIT_TIME);
  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    for (uint8_t i = 0; i < 32; i++) {
      instruct_motor(i);
    }
    vTaskDelayUntil(&xLastWakeTime, MOTOR_CONTROL_TIME); //使用vTaskDelayUntil()保证精确延迟
  }
}

int16_t max_time=10;
uint16_t speed=1100;

void quick_test_task(void *argument) {
  /* USER CODE BEGIN test_throw */
  vTaskDelay(2000);
  motor_clear_cumulative_turn(0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  vTaskDelay(5000);
  static int16_t times = 10;
  while (1) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == SET||times!=max_time) {
      if (times-- > 0) {
        motor_set_current(1, 16384);
      } else {
        motor_set_current(1, -16384);
        if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == RESET&&times<-3*max_time){
          times = max_time;
        }
      }
    } else {
      motor_set_current(1, -2000);
      times = max_time;
    }
    vTaskDelay(5);
  }

  /* USER CODE END test_throw */
}
