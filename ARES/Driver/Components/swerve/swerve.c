/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     swerve.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    舵轮驱动
  * @version  0.1
  * @date     2021-05-06
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "swerve.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "math.h"

/*
坐标系定义
        X
        ↑
        |
Y ←---- ✛ -----
        |
        |
同时逆时针为旋转正
*/

uint8_t Swerve_init(Swerve *swerve, fp32 *encoder, Motor *steerMotor, Motor *driveMotor, fp32 steerMotorRatio,
                    fp32 driveMotorRatio, fp32 x, fp32 y, fp32 encoderCenter);
void    Swerve_execute(Swerve *swerve, uint32_t timeout);
void Swerve_calc(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz);
uint8_t Swerve_goZero(Swerve *swerve);

/**
  * @brief    舵轮结构体初始化
  * 
  * @param    swerve    舵轮结构体
  * @param    encoder   编码器指针
  * @param    steerMotor 转向电机
  * @param    driveMotor 驱动电机
  * @param    steerMotorRatio 转向电机减速比
  * @param    speedRatio 从期望线速度到驱动电机转速的比例
  * @param    x         安装位置x
  * @param    y         安装位置y
  * @param    encoderCenter 编码器零点
  * @return   uint8_t   
  */
uint8_t Swerve_init(Swerve *swerve, fp32 *encoder, Motor *steerMotor, Motor *driveMotor, fp32 steerMotorRatio, fp32 speedRatio,
                    fp32 x, fp32 y, fp32 encoderCenter) {
  if (steerMotor && driveMotor) {
    swerve->steerMotor    = steerMotor;
    swerve->driveMotor    = driveMotor;
    swerve->encoderCenter = encoderCenter;
    swerve->speedRatio    = speedRatio;
    swerve->rho           = __sqrtf(x * x + y * y);
    swerve->theta         = atan2_fast(x, y);
    RM_Motor_setAsM3508(swerve->steerMotor);
    swerve->steerMotor->reductionRatio = steerMotorRatio;
    swerve->encoder                    = encoder;
    //Swerve_zero(swerve);
    swerve->zeroed = 0;
    //Swerve_update(swerve);

    Motor_Zero(swerve->steerMotor);

    return 1;
  } else {
    return 0;
  }
}

/**
  * @brief    计算舵轮运动学
  * 
  * @param    swerve    舵轮结构体
  * @param    centerVx  中心前后速度(m/s)
  * @param    centerVy  中心左右速度(m/s)
  * @param    centerWz  中心旋转速度(rad/s)
  */
void Swerve_calc(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz) {

  //第一步 得出舵轮的vx和vy角度
  fp32 swerveVx = centerVx - swerve->rho * centerWz * arm_sin_f32(swerve->theta);
  fp32 swerveVy = centerVy + swerve->rho * centerWz * arm_cos_f32(swerve->theta);

  //第二步 转化为极坐标
  swerve->speedTarget = _sqrtf(swerveVx * swerveVx + swerveVy * swerveVy);
  //swerve->angleTarget = swerve->steerMotor->status.angleOutput;
  if (swerve->speedTarget != 0) {
    swerve->angleTarget = atan2_fast(swerveVx, swerveVy);
  }
  //第三步 转过最小角度 以及速度投影
  fp32 cosErr = arm_cos_f32(swerve->angleTarget - swerve->steerMotor->status.angleOutput);

  while (swerve->angleTarget - swerve->steerMotor->status.angleOutput >= PI / 2) {
    swerve->angleTarget -= PI;
  }
  while (swerve->angleTarget - swerve->steerMotor->status.angleOutput < -PI / 2) {
    swerve->angleTarget += PI;
  }

  //误差角超过一定限度会导致期望速度无限大 转到45°内才开始速度投影
  if (cosErr < 0.70710678118654752440084436210485f && cosErr >= 0) {
    cosErr = 1.0f;
  } else if (cosErr > -0.70710678118654752440084436210485f && cosErr < 0) {
    cosErr = -1.0f;
  }
  swerve->speedTarget = swerve->speedTarget / cosErr;
}

/**
  * @brief    设定舵轮角度与速度
  * 
  * @param    swerve    舵轮结构体
  * @param    angle     设定角度
  * @param    speed     设定速度
  */
void Swerve_execute(Swerve *swerve, uint32_t timeout) {
  Motor_SetAngle(swerve->steerMotor, swerve->angleTarget, timeout);
  Motor_SetSpeed(swerve->driveMotor, swerve->speedTarget * swerve->speedRatio, timeout);
}

/**
  * @brief    舵轮回到零点
  * 
  * @param    swerve    舵轮结构体
  * 
  */
uint8_t Swerve_goZero(Swerve *swerve) {
  if (__fabs(swerve->encoderCenter - swerve->steerMotor->status.angleOutput) < 0.001) {
    Motor_Zero(swerve->steerMotor);
    swerve->zeroed = 1;
  } else {
    Motor_SetSpeed(swerve->steerMotor, 10 * (*swerve->encoder - swerve->encoderCenter), 2);
  }
  return swerve->zeroed;
}
