/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     quintic_trajectory.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    五次多项式轨迹生成
  * @version  0.1
  * @date     2021-03-17
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef QUINTIC_TRAJECTORY_H
#define QUINTIC_TRAJECTORY_H
#include "main.h"

typedef struct {
  fp32     a[6];
  int32_t  timebase;
  uint32_t t;
} QuinticTraj;

#define TRAJ_DIR_FORWARD 1
#define TRAJ_DIR_BACKWARD -1

extern void quinticTraj_init(QuinticTraj *traj, fp32 s0, fp32 v0, fp32 a0, fp32 st, fp32 vt, fp32 at, uint32_t ms);
extern void quinticTraj_start(QuinticTraj *traj);
extern void quinticTraj_calc(QuinticTraj *traj, int8_t dir, fp32 *s, fp32 *v, fp32 *a);

#endif
