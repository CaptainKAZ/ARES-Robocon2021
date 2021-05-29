/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     bezier_trajectory.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    贝塞尔曲线轨迹规划(目前只实现了5阶贝塞尔曲线)
  * @version  0.1
  * @date     2021-05-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef BEZIER_TRAJECTORY
#define BEZIER_TRAJECTORY
#include "main.h"
#include "quintic_trajectory.h"

typedef struct {
  fp32     b[6];
  int32_t  timebase;
  fp32 t;
} BezierTraj;

extern void bezierTraj_init(BezierTraj *traj, fp32 *points, uint32_t ms);
extern void bezierTraj_start(BezierTraj *traj);
extern void bezierTraj_calc(BezierTraj *traj, int8_t dir, fp32 *s, fp32 *v, fp32 *a);
#endif
