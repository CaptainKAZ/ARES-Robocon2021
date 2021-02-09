/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     imu_comm.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    imu的通信层
  * @version  0.1
  * @date     2021-01-21
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef IMU_COMM_H
#define IMU_COMM_H

#include "main.h"
#include "spi.h"

#define MPU_NSS_H() HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET)
#define MPU_NSS_L() HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET)
#define IST_RST_H() HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_SET)
#define IST_RST_L() HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_RESET)

#define SPI_WAIT_TIME 2
#define IIC_WAIT_TIME 1

#define MPU_READ_MSB 0x80
#define IST_READ_MSB 0x80

//陀螺仪读取，写入寄存器地址的数据
extern void    mpu_write_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu_read_reg(uint8_t reg);
extern void    mpu_write_regs(uint8_t reg, uint8_t *buf, uint8_t len);
extern void    mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len);
extern void    ist_auto_comm(void);
extern uint8_t ist_read_reg(uint8_t reg);
extern void    ist_write_reg(uint8_t reg, uint8_t data);
extern void    ist_read_regs(uint8_t reg, uint8_t *buf, uint8_t len);
extern void    ist_write_regs(uint8_t reg, uint8_t *data, uint8_t len);

#endif
