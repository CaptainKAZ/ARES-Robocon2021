/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     imu_comm.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    imu的通信层
  * @version  0.1
  * @date     2021-01-21
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */

#include "imu_comm.h"
#include "FreeRTOS.h"
#include "mpu6500reg.h"
#include "ist8310reg.h"

static uint8_t tx, rx;

static uint8_t mpu_rw(uint8_t byte) {
  uint8_t ret;
  if (HAL_OK == HAL_SPI_TransmitReceive(&hspi5, &byte, &ret, 1, SPI_WAIT_TIME))
    return ret;
  else
    return 0;
}

void mpu_write_reg(uint8_t reg, uint8_t data) {
  MPU_NSS_L();
  tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, SPI_WAIT_TIME);
  tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, SPI_WAIT_TIME);
  MPU_NSS_H();
}

uint8_t mpu_read_reg(uint8_t reg) {
  MPU_NSS_L();
  tx = reg | 0x80;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, SPI_WAIT_TIME);
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, SPI_WAIT_TIME);
  MPU_NSS_H();
  return rx;
}

void mpu_write_regs(uint8_t reg, uint8_t *buf, uint8_t len) {
  MPU_NSS_L();
  mpu_rw(reg);
  if (len != 0) {
    uint8_t i;
    for (i = 0; i < len; i++) {
      mpu_rw(*buf);
      buf++;
    }
  }
  MPU_NSS_H();
}

void mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len) {
  MPU_NSS_L();
  mpu_rw(reg | MPU_READ_MSB);
  if (len != 0) {
    uint8_t i;
    for (i = 0; i < len; i++) {
      *buf = mpu_rw(0xFF);
      buf++;
    }
  }
  MPU_NSS_H();
}

void ist_auto_comm(void) {
  uint8_t readBuf[3] = {IST8310_ADDRESS | IST_READ_MSB, 0x02, 0x88};
  mpu_write_regs(MPU_I2CSLV0_ADDR, readBuf, 3);
}

uint8_t ist_read_reg(uint8_t reg) {
  uint8_t readBuf[3] = {IST8310_ADDRESS | IST_READ_MSB, reg, 0x81};
  mpu_write_regs(MPU_I2CSLV0_ADDR, readBuf, 3);
  HAL_Delay(IIC_WAIT_TIME);
  return mpu_read_reg(MPU_EXT_SENS_DATA_00);
}

void ist_write_reg(uint8_t reg, uint8_t data) {
  uint8_t writeBuf[4] = {IST8310_ADDRESS, reg, data, 0x80};
  mpu_write_regs(MPU_I2CSLV4_ADDR, writeBuf, 4);
}

void ist_read_regs(uint8_t reg, uint8_t *buf, uint8_t len) {
  while (len) {
    (*buf) = ist_read_reg(reg);
    reg++;
    buf++;
    len--;
  }
}
void ist_write_regs(uint8_t reg, uint8_t *data, uint8_t len) {
  while (len) {
    ist_write_reg(reg, (*data));
    reg++;
    data++;
    len--;
    HAL_Delay(IIC_WAIT_TIME);
  }
}
