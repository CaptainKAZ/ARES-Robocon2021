#include "user_lib.h"
#include "arm_math.h"
#include <float.h>

//快速开方
fp32 invSqrt(fp32 num) {
  fp32 halfnum = 0.5f * num;
  fp32 y       = num;
  long i       = *(long *)&y;
  i            = 0x5f3759df - (i >> 1);
  y            = *(fp32 *)&i;
  y            = y * (1.5f - (halfnum * y * y));
  return y;
}

/**
  * @brief    计算二维空间中两点之间的距离
  * 
  * @param    x1        点1的x坐标
  * @param    y1        点1的y坐标
  * @param    x2        点2的x坐标
  * @param    y2        点2的y坐标
  * @return   fp32      两点之间的距离
  */
fp32 distance_2d(fp32 x1, fp32 y1, fp32 x2, fp32 y2) { return 1 / invSqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)); }


/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min) {
  ramp_source_type->frame_period = frame_period;
  ramp_source_type->max_value    = max;
  ramp_source_type->min_value    = min;
  ramp_source_type->input        = 0.0f;
  ramp_source_type->out          = 0.0f;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s
 * 即一秒后增加输入的值
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @param[in]      滤波参数
 * @retval         返回空
 */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input) {
  ramp_source_type->input = input;
  ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
  if (ramp_source_type->out > ramp_source_type->max_value) {
    ramp_source_type->out = ramp_source_type->max_value;
  } else if (ramp_source_type->out < ramp_source_type->min_value) {
    ramp_source_type->out = ramp_source_type->min_value;
  }
}
/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num[0]       = num[0];
  first_order_filter_type->input        = 0.0f;
  first_order_filter_type->out          = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
      first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit) {
  if (*num > Limit) {
    *num = Limit;
  } else if (*num < -Limit) {
    *num = -Limit;
  }
}

//判断符号位
fp32 sign(fp32 value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }
  return Value;
}

// int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }
  return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    fp32 len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    fp32 len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

__INLINE fp32 fminf(fp32 x, fp32 y) {
  if (x < y) {
    return x;
  } else {
    return y;
  }
}

__INLINE fp32 fmaxf(fp32 x, fp32 y) {
  if (x > y) {
    return x;
  } else {
    return y;
  }
}

fp32 atan2_fast(fp32 x, fp32 y) {
  float ax = fabs(x), ay = fabs(y);
  float a = fmin(ax, ay) / (fmax(ax, ay) + FLT_EPSILON);
  float s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (ay > ax)
    r = 1.57079637f - r;
  if (x < 0)
    r = 3.14159274f - r;
  if (y < 0)
    r = -r;
  return r;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang) { return loop_fp32_constrain(Ang, -180.0f, 180.0f); }

inline int is_in_isr(void) {
  volatile int tmp = 0;
  __asm
  {
    MRS tmp,IPSR
  }
  return tmp & 0x1f;
}
