/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     s_curve.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    S形轨迹实时规划器
  * @version  0.1
  * @date     2021-05-17
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef S_CURVE
#define S_CURVE
#include "main.h"
#include "stopwatch.h"

typedef struct {
  struct {
    fp32 dt[2];
    fp32 q[2];
    fp32 v[2];
    fp32 a;
  } now;
  struct {
    fp32 vMax;
    fp32 aMax;
    fp32 jMax;
  } constrain;
  struct {
    fp32 q;
    fp32 v;
    fp32 a;
    fp32 j;
  } control;
  struct {
    fp32 q;
    fp32 v;
    fp32 a;
  } error;
  struct {
    fp32 v;
    fp32 a;
  } errorConstrain;
  StopWatch stopWatch;
} SCurvePlanner;

extern void SCurvePlanner_init(SCurvePlanner *planner, fp32 vMax, fp32 aMax, fp32 jMax);
extern void SCurevePlanner_calc(SCurvePlanner *planner, fp32 *qNow, fp32 qTarget);
#endif
