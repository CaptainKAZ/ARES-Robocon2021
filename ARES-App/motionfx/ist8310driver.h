/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     ist8310driver.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    IST8310Çý¶¯
  * @version  0.1
  * @date     2020-12-06
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H
#include "main.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern void ist8310_get_data(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(fp32 mag[3]);
#endif
