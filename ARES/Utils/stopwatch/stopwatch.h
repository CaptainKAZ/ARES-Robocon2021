/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     stopwatch.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    通过TIM6来进行计时，每个计时器对象通过链表链接在一起，在TIM6中断时(10ms)更新
  * @version  0.1
  * @date     2021-02-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

#ifndef STOPWATCH_H
#define STOPWATCH_H
#include "main.h"

typedef struct StopWatch {
  volatile int32_t      last_us;        //计时开始时候TIM6_CNT的值
  volatile int32_t      dms : 31;       //每次TIM6上溢中断(10ms)的时候+1
  volatile uint8_t      enable : 1;     //是否开启该定时器
  struct StopWatch *next_stopwatch; //指向下一个定时器的指针
} StopWatch;

extern void Stopwatch_register(StopWatch *stopwatch);
extern void Stopwatch_tic(StopWatch *stopwatch);
extern fp32 Stopwatch_toc(StopWatch *stopwatch);
extern fp32 Stopwatch_disable(StopWatch *stopwatch);
extern void Stopwatch_hook(void);

#endif
