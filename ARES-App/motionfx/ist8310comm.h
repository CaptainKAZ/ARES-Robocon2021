/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     ist8310comm.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    用于与IST8310通信的头文件
  * @version  0.1
  * @date     2020-12-06
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#ifndef IST8310COMM_H
#define IST8310COMM_H

#include "main.h"

#define IST8310_IIC_ADDRESS 0x0E  //IST8310的IIC地址
#define IST8310_IIC_READ_MSB 0x80 //IST8310的SPI读取发送第一个bit为1

#define IST8310_RST_H() HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_SET)
#define IST8310_RST_L() HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_RESET)
#define IIC_DELAY 2

extern void ist8310_auto_com_by_mpu6500(void);
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

#endif
