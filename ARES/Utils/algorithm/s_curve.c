/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     s_curve.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    S形轨迹实时规划器
  * @version  0.1
  * @date     2021-05-17
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "s_curve.h"
#include "string.h"
#include "user_lib.h"
#include "arm_math.h"
#include "math.h"

void SCurvePlanner_init(SCurvePlanner *planner, fp32 vMax, fp32 aMax, fp32 jMax) {
  memset(planner, 0, sizeof(SCurvePlanner));
  Stopwatch_register(&planner->stopWatch);
  planner->constrain.aMax   = aMax;
  planner->constrain.jMax   = jMax;
  planner->constrain.vMax   = vMax;
  planner->errorConstrain.a = aMax / jMax;
  planner->errorConstrain.v = vMax / jMax;
}

static void SCurvePlanner_update(SCurvePlanner *planner, fp32 *qNow) {
  planner->now.dt[1] = planner->now.dt[0];
  planner->now.dt[0] = Stopwatch_disable(&planner->stopWatch);
  if (qNow) {
    planner->now.q[1] = planner->now.q[0];
    planner->now.q[0] = *qNow;
    planner->now.v[1] = planner->now.v[0];
    planner->now.v[0] = (planner->now.q[0] - planner->now.q[1]) / planner->now.dt[0];
    planner->now.a    = (planner->now.v[0] - planner->now.v[1]) / (0.5 * (planner->now.dt[0] + planner->now.dt[1]));
  } else {
    planner->now.a    = planner->now.a + planner->now.dt[0] * planner->control.j;
    planner->now.v[1] = planner->now.v[0];
    planner->now.v[0] = planner->now.v[0] + planner->now.dt[0] * planner->now.a;
    planner->now.q[1] = planner->now.q[0];
    planner->now.q[0] = planner->now.q[0] + planner->now.dt[0] * planner->now.v[0];
  }
  Stopwatch_tic(&planner->stopWatch);
}

void SCurevePlanner_calc(SCurvePlanner *planner, fp32 *qNow, fp32 qTarget) {
  SCurvePlanner_update(planner, qNow);
  planner->error.q = (planner->now.q[0] - qTarget) / planner->constrain.jMax;
  planner->error.v = (planner->now.v[0]) / planner->constrain.jMax;
  planner->error.a = planner->now.a / planner->constrain.jMax;
  nonLinearFilter(planner);
}

/**
  * @brief    MATLAB生成的滤波器代码，不用尝试去理解
  * 
  * @param    planner   规划器结构体
  */
static inline void nonLinearFilter(SCurvePlanner *planner) {
#define Ts (planner->now.dt[0])
#define ddemax (planner->errorConstrain.a)
#define demax (planner->errorConstrain.v)
#define U (planner->constrain.jMax)
#define ek (planner->error.q)
#define dek (planner->error.v)
#define ddek (planner->error.a)
#define qkNow (planner->now.q[0])
#define dqkNow (planner->now.v[0])
#define ddqkNow (planner->now.a)

  float   SIGMA;
  float   b_x;
  float   delta;
  float   delta_tmp;
  float   deltav_demax;
  float   ua_ddemax;
  float   x;
  uint8_t guard1 = FALSE;

  /* 'nonLinearFilter:2' delta = dek + abs(ddek)*ddek/2; */
  delta_tmp = __fabsf(ddek) * ddek;
  delta     = dek + delta_tmp / 2.0F;

  /* 'nonLinearFilter:3' if (ddek <= ddemax) && (dek <= ddek^2/2 - ddemax^2) */
  guard1 = FALSE;
  if (ddek <= ddemax) {
    SIGMA = ddek * ddek;
    if (dek <= SIGMA / 2.0F - ddemax * ddemax) {
      /* 'nonLinearFilter:4' SIGMA = ek - ddemax *(ddek^2 - 2*dek)/4 - (ddek^2 - 2*dek)^2/(8*ddemax) - ddek*(3*dek - ddek^2)/3; */
      deltav_demax = SIGMA - 2.0F * dek;
      SIGMA        = ((ek - ddemax * deltav_demax / 4.0F) - deltav_demax * deltav_demax / (8.0F * ddemax)) -
              ddek * (3.0F * dek - SIGMA) / 3.0F;
    } else {
      guard1 = FALSE;
    }
  } else {
    guard1 = FALSE;
  }

  if (guard1) {
    if ((ddek >= -ddemax) && (dek >= ddemax * ddemax - ddek * ddek / 2.0F)) {
      /* 'nonLinearFilter:5' elseif (ddek >= -ddemax) && (dek >= ddemax^2 - ddek^2/2) */
      /* 'nonLinearFilter:6' SIGMA = ek - (-ddemax)*(ddek^2 + 2*dek)/4 - (ddek^2 + 2*dek)^2/(8*(-ddemax)) + ddek*(3*dek + ddek^2)/3; */
      SIGMA        = ddek * ddek;
      deltav_demax = SIGMA + 2.0F * dek;
      SIGMA        = ((ek - -ddemax * deltav_demax / 4.0F) - deltav_demax * deltav_demax / (8.0F * -ddemax)) +
              ddek * (3.0F * dek + SIGMA) / 3.0F;
    } else {
      /* 'nonLinearFilter:7' else */
      /* 'nonLinearFilter:8' SIGMA = ek + dek*ddek*sign(delta) - ddek^3*(1 - 3*abs(sign(delta)))/6 + sign(delta)/4*sqrt(2*(ddek^2 + 2* dek*sign(delta))^3); */
      SIGMA = SIGN(delta);

      x = SIGN(delta);

      SIGMA = ((ek + dek * ddek * SIGMA) - powf(ddek, 3.0F) * (1.0F - 3.0F * __fabsf(SIGMA)) / 6.0F) +
              SIGMA / 4.0F * sqrtf(2.0F * powf(ddek * ddek + 2.0F * dek * x, 3.0F));
    }
  }

  /* 'nonLinearFilter:10' uc = -U*sign(SIGMA + (1 - abs(sign(SIGMA)))*(delta + (1 - abs(sign(delta))*ddek))); */
  x = SIGN(SIGMA);

  b_x = SIGN(delta);

  x = SIGMA + (1.0F - __fabsf(x)) * (delta + (1.0F - __fabsf(b_x) * ddek));
  if (x < 0.0F) {
    x = -1.0F;
  } else {
    if (x > 0.0F) {
      x = 1.0F;
    }
  }

  /* 'nonLinearFilter:11' ua_ddemin = -U*sign(ddek + ddemax); */
  b_x = (ddek + ddemax);
  if (b_x < 0.0F) {
    b_x = -1.0F;
  } else {
    if (b_x > 0.0F) {
      b_x = 1.0F;
    }
  }

  delta = -U * b_x;

  /* 'nonLinearFilter:12' ua_ddemax = -U*sign(ddek - ddemax); */
  b_x = (ddek - ddemax);
  if (b_x < 0.0F) {
    b_x = -1.0F;
  } else {
    if (b_x > 0.0F) {
      b_x = 1.0F;
    }
  }

  ua_ddemax = -U * b_x;

  /* 'nonLinearFilter:14' deltav_demin = ddek*abs(ddek) + 2*(dek + demax); */
  SIGMA = delta_tmp + 2.0F * (dek + demax);

  /* 'nonLinearFilter:15' deltav_demax = ddek*abs(ddek) + 2*(dek - demax); */
  deltav_demax = delta_tmp + 2.0F * (dek - demax);

  /* 'nonLinearFilter:17' ucv_demin = -U*sign(deltav_demin + (1 - abs(sign(deltav_demin)))*ddek); */
  b_x = SIGN(SIGMA);

  b_x = SIGMA + (1.0F - __fabsf(b_x)) * ddek;
  if (b_x < 0.0F) {
    b_x = -1.0F;
  } else {
    if (b_x > 0.0F) {
      b_x = 1.0F;
    }
  }

  /* 'nonLinearFilter:18' ucv_demax = -U*sign(deltav_demax + (1 - abs(sign(deltav_demax)))*ddek); */
  SIGMA = SIGN(deltav_demax);

  SIGMA = deltav_demax + (1.0F - __fabsf(SIGMA)) * ddek;
  if (SIGMA < 0.0F) {
    SIGMA = -1.0F;
  } else {
    if (SIGMA > 0.0F) {
      SIGMA = 1.0F;
    }
  }

  /* 'nonLinearFilter:20' uv_demin = max(ua_ddemin,min(ucv_demin,ua_ddemax)); */
  /* 'nonLinearFilter:21' uv_demax = max(ua_ddemin,min(ucv_demax,ua_ddemax)); */
  /* 'nonLinearFilter:23' uk = max(uv_demin,min(uc,uv_demax)); */
  /* 'nonLinearFilter:25' ddqk = ddqkNow + Ts*uk; */
  planner->control.j =
      fmaxf(fmaxf(delta, fminf(-U * b_x, ua_ddemax)), fminf(-U * x, fmaxf(delta, fminf(-U * SIGMA, ua_ddemax))));
  planner->control.a = ddqkNow + Ts * planner->control.j;

  /* 'nonLinearFilter:26' dqk = dqkNow + Ts/2*(ddqk+ddqkNow); */
  SIGMA              = Ts / 2.0F;
  planner->control.v = dqkNow + SIGMA * (planner->control.a + ddqkNow);

  /* 'nonLinearFilter:27' qk = qkNow + Ts/2*(dqk+ dqkNow); */
  planner->control.q = qkNow + SIGMA * (planner->control.v + dqkNow);

#undef Ts
#undef ddemax
#undef demax
#undef U
#undef ek
#undef dek
#undef ddek
#undef qkNow
#undef dqkNow
#undef ddqkNow
}
