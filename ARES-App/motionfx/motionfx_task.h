/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     motionfx_task.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    MotionFX任务头文件
  * @version  0.1
  * @date     2020-12-05
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#ifndef MOTIONFX_TASK_H
#define MOTIONFX_TASK_H

#define MOTIONFX_INIT_TIME 5
#define SPI_BUF_SIZE 23

extern void motionfx_task(void *pvParameters);

#endif
