/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     flash_io.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    flash读写封装
  * @version  0.1
  * @date     2021-03-07
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "flash_io.h"
#include <string.h>
uint32_t SectorError; //FLASH错误标志，全局可见

static uint32_t Flash_GetSector(const uint32_t addr) {
  uint32_t sector = 0;

  if ((addr < ADDR_FLASH_SECTOR_1) && (addr >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
  } else if ((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
  } else if ((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
  } else if ((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
  } else if ((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
  } else if ((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
  } else if ((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
  } else if ((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
  } else if ((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
  } else if ((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
  } else if ((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
  } else if ((addr < ADDR_FLASH_SECTOR_12) && (addr >= ADDR_FLASH_SECTOR_11)) {
    sector = FLASH_SECTOR_11;
  } else if ((addr < ADDR_FLASH_SECTOR_13) && (addr >= ADDR_FLASH_SECTOR_12)) {
    sector = FLASH_SECTOR_12;
  } else if ((addr < ADDR_FLASH_SECTOR_14) && (addr >= ADDR_FLASH_SECTOR_13)) {
    sector = FLASH_SECTOR_13;
  } else if ((addr < ADDR_FLASH_SECTOR_15) && (addr >= ADDR_FLASH_SECTOR_14)) {
    sector = FLASH_SECTOR_14;
  } else if ((addr < ADDR_FLASH_SECTOR_16) && (addr >= ADDR_FLASH_SECTOR_15)) {
    sector = FLASH_SECTOR_15;
  } else if ((addr < ADDR_FLASH_SECTOR_17) && (addr >= ADDR_FLASH_SECTOR_16)) {
    sector = FLASH_SECTOR_16;
  } else if ((addr < ADDR_FLASH_SECTOR_18) && (addr >= ADDR_FLASH_SECTOR_17)) {
    sector = FLASH_SECTOR_17;
  } else if ((addr < ADDR_FLASH_SECTOR_19) && (addr >= ADDR_FLASH_SECTOR_18)) {
    sector = FLASH_SECTOR_18;
  } else if ((addr < ADDR_FLASH_SECTOR_20) && (addr >= ADDR_FLASH_SECTOR_19)) {
    sector = FLASH_SECTOR_19;
  } else if ((addr < ADDR_FLASH_SECTOR_21) && (addr >= ADDR_FLASH_SECTOR_20)) {
    sector = FLASH_SECTOR_20;
  } else if ((addr < ADDR_FLASH_SECTOR_22) && (addr >= ADDR_FLASH_SECTOR_21)) {
    sector = FLASH_SECTOR_21;
  } else if ((addr < ADDR_FLASH_SECTOR_23) && (addr >= ADDR_FLASH_SECTOR_22)) {
    sector = FLASH_SECTOR_22;
  } else if (addr < ADDR_FLASH_SECTOR_END && (addr >= ADDR_FLASH_SECTOR_23)) {
    sector = FLASH_SECTOR_23;
  }
  return sector;
}
/**
  * @brief    使用Flash存储数据
  * 
  * @param    data      内存中数据地址
  * @param    len       内存中数据长度
  * @return   uint8_t   成功/失败
  */
uint8_t Flash_Save(uint8_t *data, uint32_t len) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  /*erase flash before program */
  uint32_t WriteSector = Flash_GetSector(PARAM_SAVED_START_ADDRESS);
  uint32_t start_addr  = PARAM_SAVED_START_ADDRESS;
  HAL_FLASH_Unlock();

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector       = WriteSector;
  EraseInitStruct.NbSectors    = 1;
  while (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError))
    ;

  /*start program flash*/
  while (len--) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_addr, *data);
    start_addr++;
    data++;
  }
  /*write data end*/
  HAL_FLASH_Lock();
  return 0;
}

/**
  * @brief    从Flash中拷贝数据
  * 
  * @param    buff      需要拷贝到的内存地址
  * @param    len       拷贝长度
  */
void Flash_Get(uint8_t *buff, uint32_t len) { memcpy(buff, (void*)PARAM_SAVED_START_ADDRESS, len); }
