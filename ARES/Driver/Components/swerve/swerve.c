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

uint8_t Swerve_init(Swerve *swerve, CanEncoder *encoder, Motor *steerMotor, Motor *driveMotor, int32_t encoderRatio,
                    fp32 steerMotorRatio, fp32 driveMotorRatio, fp32 x, fp32 y, PID_ControllerParam *steerPidParam);
void    Swerve_update(Swerve *swerve);
void    Swerve_calcKinematics(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz);
void    Swerve_setTargets(Swerve *swerve, fp32 angle, fp32 speed);
void    Swerve_execute(Swerve *swerve, int32_t timeout);

static MotorInstructType Swerve_controllerUpdate(Motor *motor, Controller *controller, void *param) {
#define SWERVE ((Swerve *)param)
  (void)controller;
  if (xTaskGetTickCount() - SWERVE->updateTime > 1) {
    Swerve_update(SWERVE);
  }
  //TODO:如果新的编码器信息没来，就不更新rpm设定
  fp32 rpmset =
      controllerUpdate((Controller *)&(((RM_Motor *)motor)->angle_pid), &SWERVE->steerTarget, &SWERVE->steerAngle, NULL);
  ((RM_Motor *)motor)->set_current =
      controllerUpdate((Controller *)&(((RM_Motor *)motor)->speed_pid), &rpmset, &motor->status.speed, NULL);

  return INSTRUCT_CURRENT;

#undef SWERVE
}

/**
  * @brief    舵轮初始化
  * 
  * @param    swerve    舵轮结构体
  * @param    encoder   舵轮编码器指针
  * @param    steerMotor 转向电机指针
  * @param    driveMotor 航向电机指针
  * @param    encoderRatio 编码器减速比
  * @param    steerMotorRatio 舵向电机减速比
  * @param    driveMotorRatio 航向电机减速比
  * @param    x         相对于中心的x位置(前为x正)
  * @param    y         相对于中心的y位置(左为y正)
  * @param    pidParam  转向电机的PID
  * 
  * @retval   电机是否有效
  * 
  * @note     转向电机的角度环PID直接被用作转向机构整体的PID 如果转向不使用RM电机要改
  */
uint8_t Swerve_init(Swerve *swerve, CanEncoder *encoder, Motor *steerMotor, Motor *driveMotor, int32_t encoderRatio,
                    fp32 steerMotorRatio, fp32 driveMotorRatio, fp32 x, fp32 y, PID_ControllerParam *steerPidParam) {
  if (steerMotor && driveMotor) {
    swerve->steerMotor      = steerMotor;
    swerve->driveMotor      = driveMotor;
    swerve->encoderRatio    = encoderRatio;
    swerve->steerMotorRatio = steerMotorRatio;
    swerve->driveMotorRatio = driveMotorRatio;
    swerve->steerTarget     = 0;
    swerve->rho             = __sqrtf(x * x + y * y);
    swerve->theta           = atan2_fast(x, y);
#if SWERVE_EXTERNAL_ENCODER
    //TODO:使用Monitor判断编码器是否上线
    swerve->encoder = encoder;
    while (encoder->updateTime == 0) {
      vTaskDelay(10);
      if (swerve->encoder) {
        for (uint8_t j = 0; j < 2; j++) {
          for (uint8_t i = 0; i < 4; i++) {
            if (swerve->encoder == &canEncoder[j][i]) {
              Encoder_readValue(j, i + 2);
              break;
            }
          }
        }
      }
    }
#else
    swerve->encoder = NULL;
    Swerve_zero(swerve);
#endif

    Swerve_update(swerve);

    Motor_SetAnglePID(swerve->steerMotor, steerPidParam);
    Motor_SetAltController(swerve->steerMotor, NULL, swerve, Swerve_controllerUpdate);
    return 1;
  } else {
    return 0;
  }
}

/**
  * @brief    更新舵轮结构体
  * 
  * @param    swerve    舵轮结构体

  */
void Swerve_update(Swerve *swerve) {
#if SWERVE_EXTERNAL_ENCODER
  if (swerve->encoderRatio > 0) {
    swerve->steerAngle =
        (fp32)(((swerve->encoder->value)) % (swerve->encoderRatio)) / (fp32)(swerve->encoderRatio) * 2.0f * PI;
  } else {
    swerve->steerAngle =
        2.0f * PI - (fp32)(((swerve->encoder->value)) % (-swerve->encoderRatio)) / (fp32)(-swerve->encoderRatio) * 2.0f * PI;
  }
#else
  swerve->steerAngle = fmodf((swerve->steerMotor->status.cumulativeTurn * 2.0f * PI + swerve->steerMotor->status.angle -
                              swerve->steerMotor->status.zero) /
                                 swerve->steerMotorRatio,
                             2*PI);
#endif
  swerve->driveSpeed = swerve->driveMotor->status.speed * swerve->driveMotorRatio;

  swerve->updateTime = xTaskGetTickCount();
}

/**
  * @brief    计算舵轮运动学
  * 
  * @param    swerve    舵轮结构体
  * @param    centerVx  中心前后速度(m/s)
  * @param    centerVy  中心左右速度(m/s)
  * @param    centerWz  中心旋转速度(rad/s)
  */
void Swerve_calcKinematics(Swerve *swerve, fp32 centerVx, fp32 centerVy, fp32 centerWz) {

  //第一步 得出舵轮的vx和vy角度
  fp32 swerveVx = centerVx - swerve->rho * centerWz * arm_sin_f32(swerve->theta);
  fp32 swerveVy = centerVy + swerve->rho * centerWz * arm_cos_f32(swerve->theta);

  //第二步 转化为极坐标
  fp32 swerveSpeed = _sqrtf(swerveVx * swerveVx + swerveVy * swerveVy);
  fp32 swerveAngle = swerve->steerAngle;
  if (swerveSpeed != 0) {
    swerveAngle = atan2_fast(swerveVx, swerveVy);
  }

  //第三步 转过最小角度 以及速度投影
  Swerve_setTargets(swerve, swerveAngle, swerveSpeed);
}

/**
  * @brief    设定舵轮角度与速度
  * 
  * @param    swerve    舵轮结构体
  * @param    angle     设定角度
  * @param    speed     设定速度
  */
void Swerve_setTargets(Swerve *swerve, fp32 angle, fp32 speed) {
  if (xTaskGetTickCount() - swerve->updateTime > 1) {
    Swerve_update(swerve);
  }

  

  fp32 cosErr = arm_cos_f32(angle - swerve->steerAngle);

  while (angle - swerve->steerAngle >= PI / 2) {
    angle -= PI;
  }
  while (angle - swerve->steerAngle < -PI / 2) {
    angle += PI;
  }
  swerve->steerTarget = angle;

  //误差角超过一定限度会导致期望速度无限大 转到45°内才开始速度投影
  if (cosErr < 0.70710678118654752440084436210485f && cosErr >= 0) {
    cosErr = 1.0f;
  } else if (cosErr > -0.70710678118654752440084436210485f && cosErr < 0) {
    cosErr = -1.0f;
  }
  swerve->driveTarget = speed / cosErr;
}

/**
  * @brief    舵轮执行
  * 
  * @param    swerve    舵轮结构体
  * @param    timeout   超时
  */
void Swerve_execute(Swerve *swerve, int32_t timeout) {
  Motor_AltControl(swerve->steerMotor, timeout);
  Motor_SetSpeed(swerve->driveMotor, swerve->driveTarget / swerve->driveMotorRatio, timeout);
}

/**
  * @brief    舵轮设置零点
  * 
  * @param    swerve    舵轮结构体
  * 
  * @note     这实现比较低效
  */
void Swerve_zero(Swerve *swerve) {
#if SWERVE_EXTERNAL_ENCODER
  if (swerve->encoder) {
    for (uint8_t j = 0; j < 2; j++) {
      for (uint8_t i = 0; i < 4; i++) {
        if (swerve->encoder == &canEncoder[j][i]) {
          Encoder_zero(j, i + 2);
          Encoder_zero(j, i + 2);
          Encoder_setMode(j, i + 2, 0xAA);
          Encoder_setMode(j, i + 2, 0xAA);
          Encoder_setFeedbackTime(j, i + 2, 3000);
          Encoder_setFeedbackTime(j, i + 2, 3000);
          return;
        }
      }
    }
  }
#else
  Motor_Zero(swerve->steerMotor);
#endif
}
