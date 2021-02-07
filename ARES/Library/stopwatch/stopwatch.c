/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     stopwatch.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    通过TIM6来进行计时，每个计时器对象通过链表链接在一起，在TIM6中断时(10ms)更新
  * @version  0.1
  * @date     2021-02-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

#include "stopwatch.h"
#include "tim.h"
static stopwatch_t head;
stopwatch_t *      ptr;

/**
  * @brief    注册计时器
  * 
  * @param    stopwatch 待注册的计时器指针
  */
void stopwatch_register(stopwatch_t *stopwatch) {
  ptr = &head;
  //跳转到链表尾
  while (ptr->next_stopwatch != NULL) {
    ptr = ptr->next_stopwatch;
  }
  ptr->next_stopwatch       = stopwatch;
  stopwatch->dms            = 0;
  stopwatch->enable         = DISABLE;
  stopwatch->next_stopwatch = NULL;
}

/**
  * @brief    计时器开始计时
  * 
  * @param    stopwatch 计时器指针
  */
void tic(stopwatch_t *stopwatch) {
  stopwatch->last_us = htim6.Instance->CNT;
  stopwatch->dms     = 0;
  stopwatch->enable  = ENABLE;
}

/**
  * @brief    获得计时器目前时间，不会停止计时器
  * 
  * @param    stopwatch 计时器指针
  * @return   fp32      计时器时间，单位为秒
  */
fp32 toc(stopwatch_t *stopwatch) { return 1e-2 * stopwatch->dms + 1e-6 * (stopwatch->last_us - htim6.Instance->CNT); }

/**
  * @brief    停止计时器
  * 
  * @param    stopwatch 计时器指针
  * @return   fp32      计时器停止时候的时间，单位为秒
  */
fp32 stopwatch_disable(stopwatch_t *stopwatch) {
  stopwatch->enable = DISABLE;
  stopwatch->dms    = 0;
  return 1e-2 * stopwatch->dms + 1e-6 * (stopwatch->last_us - htim6.Instance->CNT);
}

/**
  * @brief    更新定时器，只能在TIM6中断中调用
  * 
  */
void stopwatch_hook(void) {
  ptr = &head;
  //遍历每一个链表
  while (head.next_stopwatch != NULL) {
    if (ptr->enable) {
      ++ptr->dms;
    }
    ptr = ptr->next_stopwatch;
  }
}
