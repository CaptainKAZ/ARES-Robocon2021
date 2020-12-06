/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mpu6500comm.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    与MPU6500通信的有关函数
  * @version  0.1
  * @date     2020-12-05
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */

#include "main.h"
#include "mpu6500comm.h"
#include "FreeRTOS.h"
#include "spi.h"

static uint8_t mpu6500_SPI_read_write_byte(uint8_t byte) {
  uint8_t ret;
  if (HAL_OK == HAL_SPI_TransmitReceive(&hspi5, &byte, &ret, 1, SPI_WAIT_TIME))
    return ret;
  else
    return 0;
}

void mpu6500_write_single_reg(uint8_t reg, uint8_t data) {
  mpu6500_SPI_NSS_L();
  mpu6500_SPI_read_write_byte(reg);
  mpu6500_SPI_read_write_byte(data);
  mpu6500_SPI_NSS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg) {
  uint8_t ret;
  mpu6500_SPI_NSS_L();
  mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
  ret = mpu6500_SPI_read_write_byte(0xFF);
  mpu6500_SPI_NSS_H();
  return ret;
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
  mpu6500_SPI_NSS_L();
  if (len != 0) {
    uint8_t i;
    for (i = 0; i < len; i++) {
      mpu6500_SPI_read_write_byte(*buf);
      buf++;
    }
  }
  mpu6500_SPI_NSS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
  mpu6500_SPI_NSS_L();
  mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
  if (len != 0) {
    uint8_t i;
    for (i = 0; i < len; i++) {
      *buf = mpu6500_SPI_read_write_byte(0xFF);
      buf++;
    }
  }
  mpu6500_SPI_NSS_H();
}
