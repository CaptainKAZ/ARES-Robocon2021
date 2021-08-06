/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     computer_usb.h
  * @author   ljy
  * @brief    计算机通信
  * @version  0.1
  * @date     2021-07-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef COMPUTER_USB_H
#define COMPUTER_USB_H
#include "main.h"
#include "string.h"
#include "interboard_spi.h"

#define COMPUTER_PACKET_MAX_SIZE 28
#define COMPUTER_MAX_PACKET_NUM 10

extern uint8_t Interboard_computerRxHook(uint8_t *buf, uint8_t len);
#ifdef MASTER_BOARD
extern void Computer_tx(uint8_t *Buf, uint32_t Len);
#elif defined SLAVE_BOARD
extern void Computer_rxHook(uint8_t *Buf, uint32_t Len);
#include "usbd_cdc_if.h"
#endif

#endif
