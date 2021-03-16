/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     led_monitor_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    LED灯语任务
  * @version  0.1
  * @date     2021-03-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "led_monitor_task.h"

static void value2led(uint8_t value) {
  if (value > 11) {
    FLOWLED_ON(0);
  } else {
    FLOWLED_OFF(0);
  }
  if (value > 22) {
    FLOWLED_ON(1);
  } else {
    FLOWLED_OFF(1);
  }
  if (value > 33) {
    FLOWLED_ON(2);
  } else {
    FLOWLED_OFF(2);
  }
  if (value > 44) {
    FLOWLED_ON(3);
  } else {
    FLOWLED_OFF(3);
  }
  if (value > 55) {
    FLOWLED_ON(4);
  } else {
    FLOWLED_OFF(4);
  }
  if (value > 66) {
    FLOWLED_ON(5);
  } else {
    FLOWLED_OFF(5);
  }
  if (value > 77) {
    FLOWLED_ON(6);
  } else {
    FLOWLED_OFF(6);
  }
  if (value >= 88) {
    FLOWLED_ON(7);
  } else {
    FLOWLED_OFF(7);
  }
}

void led_monitor_task() {
  static uint8_t value = 0;
  while (value < 100) {
    value2led(value);
    vTaskDelay(10);
    value++;
  }
  vTaskDelay(300);
  value2led(0);
  vTaskDelay(200);
  for (;;) {
    value = osGetCPUUsage();
    value2led(value);
    vTaskDelay(100);
  }
}
