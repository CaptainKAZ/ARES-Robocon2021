/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     quintic_trajactory.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    五次多项式轨迹生成
  * @version  0.1
  * @date     2021-03-17
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "quintic_trajectory.h"
#include "cmsis_os.h"

/**
  * @brief    初始化五次多项式轨迹（计算系数）
  * 
  * @param    traj      五次多项式轨迹结构体
  * @param    s0        起始位置
  * @param    v0        起始速度
  * @param    a0        起始加速度
  * @param    st        结束位置
  * @param    vt        结束速度
  * @param    at        结束加速度
  * @param    ms         持续时间(毫秒)
  */
void quinticTraj_init(quintic_traj *traj, fp32 s0, fp32 v0, fp32 a0, fp32 st, fp32 vt, fp32 at, uint32_t ms) {
#define t2 (t * t)
#define t3 (t * t * t)
#define t4 (t * t * t * t)
#define t5 (t * t * t * t * t)
#define h (st - s0)
  fp32 t         = ms;
  traj->t        = ms;
  traj->a[0]     = s0;
  traj->a[1]     = v0;
  traj->a[2]     = 0.5f * a0;
  traj->a[3]     = 0.5f * (20 * h - (8 * vt + 12 * v0) * t + (at - 3 * a0) * t2) / t3;
  traj->a[4]     = 0.5f * (-30 * h + (16 * v0 + 14 * vt) * t + (3 * a0 - 2 * at) * t2) / t4;
  traj->a[5]     = 0.5f * (12 * h - (v0 + vt) * 6 * t + (at - a0) * t2) / t5;
  traj->timebase = 0;

#undef t2
#undef t3
#undef t4
#undef t5
#undef h
}

/**
  * @brief    开始五次多项式轨迹（设置时间起点）
  * 
  * @param    traj      五次多项式轨迹结构体
  */
void quinticTraj_start(quintic_traj *traj) {
  if (xTaskGetTickCount() - traj->timebase > traj->t) {
    traj->timebase = xTaskGetTickCount();
  }
}

void quinticTraj_calc(quintic_traj *traj, int8_t dir, fp32 *s, fp32 *v, fp32 *a) {
  fp32 T;
  if (xTaskGetTickCount() - traj->timebase > traj->t) {
    T = traj->t;
  } else {
    T = xTaskGetTickCount() - traj->timebase;
  }
  if (dir == TRAJ_DIR_BACKWARD) {
    T = traj->t - T;
  }
  if (s != NULL) {
    *s = traj->a[0] + traj->a[1] * T + traj->a[2] * T * T + traj->a[3] * T * T * T + traj->a[4] * T * T * T * T +
         traj->a[5] * T * T * T * T * T;
  }
  if (v != NULL) {
    *v = dir * (traj->a[1] + 2.0f * traj->a[2] * T + 3.0f * traj->a[3] * T * T + 4.0f * traj->a[4] * T * T * T +
         5.0f * traj->a[5] * T * T * T * T);
  }
  if (a != NULL) {
    *a = 2.0f * traj->a[2] + 6.0f * traj->a[3] * T + 12.0f * traj->a[4] * T * T + 20.0f * traj->a[5] * T * T * T;
  }
}
