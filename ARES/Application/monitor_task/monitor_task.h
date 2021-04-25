/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     monitor_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    监测任务
  * @version  0.1
  * @date     2021-03-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H
#include "main.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "cpu_utils.h"

#define FLOWLED_OFF(i) HAL_GPIO_WritePin(LED_GPIO_Port, LED8_Pin >> (i), GPIO_PIN_SET)
#define FLOWLED_ON(i) HAL_GPIO_WritePin(LED_GPIO_Port, LED8_Pin >> (i), GPIO_PIN_RESET)

#define MONITOR_ERROR_EXIST ((uint32_t)1 << 31)
#define MONITOR_ERROR_LOST ((uint32_t)1 << 30)
#define MONITOR_ERROR_INVALID ((uint32_t)1 << 29)

//这里定义各个设备的显示ID
#define MONITOR_SBUS_ID 0
#define MONITOR_OPS_ID 1
#define MONITOR_MCP_ID 2

/**
  * @brief    监测节点结构体
  * 
  */
typedef struct MonitorList {

  /**
    * @brief          守护函数
    * 
    * @param errorReg 错误状态寄存器
    * 
    */
  void (*guardFun)(uint32_t *errorReg, uint32_t *errorTime);

  /**
   * @brief    状态寄存器 用于对应细分项目
   * 
   */
  uint32_t errorReg;

  /**
    * @brief    错误时间
    * 
    */
  uint32_t errorTime;

  /**
    * @brief    状态显示ID,负数为不显示
    * 
    */
  int8_t displayId;

  /**
    * @brief    下一个节点指针
    * 
    */
  struct MonitorList *next;

} MonitorList;

void Monitor_registor(MonitorList *list, void (*guardFun)(uint32_t *errorReg,uint32_t* errorTime), int8_t displayId);

#endif
