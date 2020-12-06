/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mpu6500comm.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    mpu6500通信头文件
  * @version  0.1
  * @date     2020-12-06
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#ifndef MPU6500COMM_H
#define MPU6500COMM_H

#include "main.h"
#include "mpu6500reg.h"

#define mpu6500_SPI_NSS_H() HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET)
#define mpu6500_SPI_NSS_L() HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET)
#define SPI_WAIT_TIME 2

//mpu6500 SPI读取寄存器时候，需要在寄存器地址最高 置1
#define MPU_SPI_READ_MSB 0x80

//陀螺仪读取，写入寄存器地址的数据
extern void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu6500_read_single_reg(uint8_t reg);
extern void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);


#endif
