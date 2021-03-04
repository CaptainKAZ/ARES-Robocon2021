/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     motor_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    电机控制任务，定时更新电机控制
  * @version  0.1
  * @date     2021-03-04
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "motor_task.h"

extern void RM_Motor_Execute(void);
extern void VESC_Motor_Execute(void);
extern void RMD_Motor_Execute(void);

void motor_task(void *pvParameters) {
  TickType_t xLastWakeTime;
  CAN_Start(&hcan1);
  CAN_Start(&hcan2);

  vTaskDelay(MOTOR_INIT_TIME);
  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    RM_Motor_Execute();
    VESC_Motor_Execute();
    RMD_Motor_Execute();
    vTaskDelayUntil(&xLastWakeTime, MOTOR_CTRL_TIME); //使用vTaskDelayUntil()保证精确延迟
  }
}
