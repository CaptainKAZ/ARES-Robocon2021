/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     bezier_trajectory.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    贝塞尔曲线规划(目前只实现了五次贝塞尔曲线规划)
  * @version  0.1
  * @date     2021-05-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "bezier_trajectory.h"
#include "cmsis_os.h"

/**
  * @brief    贝塞尔曲线计算系数
  * 
  * @param    traj      贝塞尔曲线结构体
  * @param    points    贝塞尔曲线控制点(长度为6)
  * @param    ms        持续时间
  */
void bezierTraj_init(BezierTraj *traj, fp32 *points, uint32_t ms) {
  traj->b[5]     = -points[0] + 5 * points[1] - 10 * points[2] + 10 * points[3] - 5 * points[4] + points[5];
  traj->b[4]     = 5 * points[0] - 20 * points[1] + 30 * points[2] - 20 * points[3] + 5 * points[4];
  traj->b[3]     = -10 * points[0] + 30 * points[1] - 30 * points[2] + 10 * points[3];
  traj->b[2]     = 10 * points[0] - 20 * points[1] + 10 * points[2];
  traj->b[1]     = -5 * points[0] + 5 * points[1];
  traj->b[0]     = points[0];
  traj->t        = ms;
  traj->timebase = 0;
}

/**
  * @brief    开始贝塞尔轨迹（设置时间起点）
  * 
  * @param    traj      五次多项式轨迹结构体
  */
void bezierTraj_start(BezierTraj *traj) {
  if (xTaskGetTickCount() - traj->timebase > traj->t) {
    traj->timebase = xTaskGetTickCount();
  }
}

/**
  * @brief    贝塞尔轨迹计算
  * 
  * @param    traj      轨迹结构体
  * @param    dir       方向
  * @param    s         返回当前的位置
  * @param    v         返回当前的速度
  * @param    a         返回当前的加速度
  */
void bezierTraj_calc(BezierTraj *traj, int8_t dir, fp32 *s, fp32 *v, fp32 *a) {
  fp32 T;
  if (xTaskGetTickCount() - traj->timebase > traj->t) {
    fp32 debug= xTaskGetTickCount() - traj->timebase;
    T = 1.0f;
  } else {
    T = (fp32)(xTaskGetTickCount() - traj->timebase) / traj->t;
  }
  if (dir == TRAJ_DIR_BACKWARD) {
    T = 1.0f - T;
  }
  if (s != NULL) {
    *s = traj->b[0] + traj->b[1] * T + traj->b[2] * T * T + traj->b[3] * T * T * T + traj->b[4] * T * T * T * T +
         traj->b[5] * T * T * T * T * T;
  }
  if (v != NULL) {
    *v = dir * (traj->b[1] + 2.0f * traj->b[2] * T + 3.0f * traj->b[3] * T * T + 4.0f * traj->b[4] * T * T * T +
                5.0f * traj->b[5] * T * T * T * T)/ traj->t;
  }
  if (a != NULL) {
    *a = 2.0f * traj->b[2] + 6.0f * traj->b[3] * T + 12.0f * traj->b[4] * T * T + 20.0f * traj->b[5] * T * T * T;
  }
}
