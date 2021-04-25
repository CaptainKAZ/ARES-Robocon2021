/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     monitor_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    监测任务
  * @version  0.1
  * @date     2021-03-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "monitor_task.h"

static MonitorList  head;
static uint8_t      monitorNum = 0;
static uint8_t      displayReg = 0xFF;
static MonitorList *ptr        = &head;

/**
  * @brief    监测节点初始化
  * 
  * @param    list      监测节点指针
  * @param    guardFun  守护函数（周期性被调用，用于发现错误，记录出错时间以及原因并尝试修复）
  * @param    displayId 在流水灯上显示的ID（负数为不显示）
  */
void Monitor_registor(MonitorList *list, void (*guardFun)(uint32_t *errorReg,uint32_t* errorTime), int8_t displayId) {
  MonitorList *ptr = &head;
  list->guardFun   = guardFun;
  list->errorReg   = 0;
  list->errorTime  = 0;
  list->displayId  = displayId;
  list->next       = NULL;
  //跳到链表末尾
  while (ptr->next != NULL) {
    ptr = ptr->next;
  }
  ptr->next = list;
  monitorNum++;
}

/**
  * @brief    通过LED可视化数值
  * 
  * @param    value     LED的数值，符号为方向
  * @param    maxValue  最大数值，符号为方向
  */
static void value2Led(fp32 value, fp32 maxValue) {
  for (int8_t i = 0; i < 8; i++) {
    if (maxValue >= 0) {
      if ((i + 1.0f) * maxValue * 0.12f <= value) {
        FLOWLED_ON(i);
      } else {
        FLOWLED_OFF(i);
      }
    } else {
      if ((i + 1.0f) * -maxValue * 0.12f <= -value) {
        FLOWLED_ON(7 - i);
      } else {
        FLOWLED_OFF(7 - i);
      }
    }
  }
}

/**
  * @brief    从8bit寄存器转化为灯语
  * 
  * @param    reg       寄存器值
  */
static void reg2Led(uint8_t reg) {
  for (uint8_t i = 0; i < 8; i++) {
    if(READ_BIT(reg,1<<i)){
      FLOWLED_ON(i);
    }else{
      FLOWLED_OFF(i);
    }
  }
}

/**
  * @brief    系统初始化启动灯语
  * 
  * @note     这个函数是阻塞的
  * 
  */
static void Monitor_initSignal(void) {
  for (int8_t i = 0; i < 100; i++) {
    value2Led(i, 100);
    vTaskDelay(10);
  }
  vTaskDelay(300);
  value2Led(0, 100);
  vTaskDelay(200);
}

void monitor_task() {

  Monitor_initSignal();

  for (;;) {
    ptr        = &head;
    displayReg = 0xFF;
    while (ptr != NULL) {

      if (ptr->guardFun) {
        ptr->guardFun(&ptr->errorReg, &ptr->errorTime);
      }
      if (ptr->displayId >= 0) {
        if (READ_BIT(ptr->errorReg, MONITOR_ERROR_EXIST)) {
          CLEAR_BIT(displayReg, 1 << (ptr->displayId));
        } else {
          SET_BIT(displayReg, 1 << (ptr->displayId));
        }
      }
      ptr = ptr->next;
    }
    if (displayReg != 0xFF) {
      reg2Led(displayReg);
    }else{
      value2Led(osGetCPUUsage(), 100);
    }

    vTaskDelay(10);
  }
}
