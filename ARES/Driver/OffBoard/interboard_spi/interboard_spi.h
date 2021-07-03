/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     interboard_spi.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    SPI板间通信协议
  * @version  0.1
  * @date     2021-06-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef INTERBOARD_SPI_H
#define INTERBOARD_SPI_H
#include "main.h"
#include "spi.h"

/**
  * @brief    通信操作部分
  * 
  */
typedef enum {
  INTERBOARD_READY,
  INTERBOARD_PENDING,
  INTERBOARD_BUSY,
} InterboardState;

/**
  * @brief    通信协议部分
  * 
  */
#define INTERBOARD_MAX_FRAME_LENGTH (30)
#define INTERBOARD_FRAME_HEAD (0x0F)
#define INTERBOARD_FRAME_ACK (0xF0)
#define INTERBOARD_TXBUF_SIZE (3)
typedef enum {
  INTERBOARDMSG_CAN,
  INTERBOARDMSG_BATT,
  INTERBOARDMSG_IMU,
  INTERBOARDMSG_COMPUTER,
} InterboardMsgType;

extern void            Interboard_start(void);
extern void            Interboard_init(void);
extern void            Interboard_tx(InterboardMsgType msgType, uint8_t len, uint8_t *buf);
extern InterboardState interboardTxState;
extern void            Interboard_txRxCpltHook(void);
extern void            Interboard_errorHook(void);

#endif
