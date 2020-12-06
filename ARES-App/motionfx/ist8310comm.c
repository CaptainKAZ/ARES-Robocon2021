/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     ist8310comm.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    用于与IST8310通信的有关函数
  * @version  0.1
  * @date     2020-12-06
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#include "ist8310comm.h"
#include "stm32f4xx.h"

#include "mpu6500comm.h"
#include "mpu6500reg.h"

void ist8310_auto_com_by_mpu6500(void) {
  uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, 0x02, 0x88};
  mpu6500_write_muli_reg(MPU_I2CSLV0_ADDR, readBuf, 3);
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg) {
  uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, reg, 0x81};
  mpu6500_write_muli_reg(MPU_I2CSLV0_ADDR, readBuf, 3);
  HAL_Delay(IIC_DELAY);
  return mpu6500_read_single_reg(MPU_EXT_SENS_DATA_00);
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
  uint8_t writeBuf[4] = {IST8310_IIC_ADDRESS, reg, data, 0x80};
  mpu6500_write_muli_reg(MPU_I2CSLV4_ADDR, writeBuf, 4);
}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
  while (len) {
    (*buf) = ist8310_IIC_read_single_reg(reg);
    reg++;
    buf++;
    len--;
  }
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len) {
  while (len) {
    ist8310_IIC_write_single_reg(reg, (*data));
    reg++;
    data++;
    len--;
    HAL_Delay(IIC_DELAY);
  }
}
