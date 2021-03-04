/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     RMD_motor.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    光毓RMD电机驱动程序
  * @version  0.1
  * @date     2021-03-03
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H
#include "main.h"
#include "motor.h"
#include "PID.h"

#define RMD_MULTI_FRAME_HEAD 0x280
#define RMD_MOTOR_ECD2RAD 1.9175930254469836040179719119084e-4f
#define RMD_MOTOR_I_FACTOR 16.11328125f
#define RMD_MOTOR_DPS2RPM 0.16666666666666666666666666666667f

typedef enum {
  RMD_READ_PID          = 0x30,
  RMD_WRITE_PID_RAM     = 0x31,
  RMD_WRITE_PID_ROM     = 0x32,
  RMD_READ_ACC          = 0x33,
  RMD_WRITE_ACC_RAM     = 0x34,
  RMD_READ_ENCODER      = 0x90,
  RMD_ZERO_SET          = 0x91,
  RMD_ZERO_NOW          = 0x19,
  RMD_READ_MULTI_TURN   = 0x92,
  RMD_READ_SINGLE_TURN  = 0x94,
  RMD_CLEAR_TURN        = 0x95,
  RMD_READ_STATE_1_FLAG = 0x9A,
  RMD_CLEAR_FLAG        = 0x9B,
  RMD_READ_STATE_2      = 0x9C,
  RMD_READ_STATE_3      = 0x9D,
  RMD_SHUTDOWN          = 0x80,
  RMD_STOP              = 0x81,
  RMD_RUN               = 0x88,
  RMD_TORQUE_LOOP       = 0xA1,
  RMD_SPEED_LOOP        = 0xA2,
  RMD_POSITION_LOOP_1   = 0xA3,
  RMD_POSITION_LOOP_2   = 0xA4,
  RMD_POSITION_LOOP_3   = 0xA5,
  RMD_POSITION_LOOP_4   = 0xA6,
  RMD_POSITION_LOOP_5   = 0xA7,
  RMD_POSITION_LOOP_6   = 0xA8,
} RMD_SINGLE_COMMAND;


typedef struct {
  Motor          general;
  uint32_t       rx_timestamp;
  PID_Controller speed_pid;
  PID_Controller angle_pid;
  fp32           set_current;
  fp32           zero;
  uint32_t       cumulative_turn;
  Controller *   alt_controller;
  fp32 (*alt_controller_update)(Motor *motor, Controller *controller);
} RMD_Motor;

#endif
