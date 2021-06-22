/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     interboard_socketspi.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    SPI板间通信协议
  * @version  0.1
  * @date     2021-06-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "interboard_spi.h"
#include "string.h"
/**
  *                                板间通信的基本流程
  * 
  * 设计思想：保证通信成功的前提下 最小化主机开销
  * 
  *    _________________________                        _________________________
  *   |           ______________|                      |______________           |
  *   |          |SPI1          |                      |          SPI2|          |
  *   |          |              |                      |              |          |
  *   |          |           CLK|______________________|CLK           |          |
  *   |          |              |                      |              |          |
  *   |          |          MISO|______________________|MISO          |          |
  *   |          |              |                      |              |          |
  *   |          |          MOSI|______________________|MOSI          |          |
  *   |          |              |                      |              |          |
  *   |          |______________|                      |______________|          |
  *   |      __                 |                      |      __                 |
  *   |     |__|             INT|______________________|INT  |__|                |
  *   |     USER             CSS|______________________|CSS  USER                |
  *   |                         |                      |                         |
  *   |               24v POWER0|                      |IN 24V                   |
  *   |                      GND|______________________|GND                      |
  *   |                         |                      |                         |
  *   |_ MASTER ________________|                      |_ SLAVE _________________|
  * 
  * 主机->从机：
  * 主机拉低CSS引脚 开始传输并接受定长51Byte 其中50byte为数据 第51Byte为ACK
  * 从机接收50Byte后尝试解码，如果接收成功发送ACK信号0xF0
  * 主机查询ACK信号，如果成功则结束发送，如果不成功则重试发送
  * 
  * 从机->主机：
  * 从机将待发送内容填入BUFFER
  * 从机拉低INT引脚告诉主机有内容待发送
  * 主机拉低CSS引脚 开始传输并接受定长50Byte数据
  * 主机接收50Byte后尝试解码，如果接收成功发送ACK信号0xF0
  * 从机查询ACK信号，如果成功则结束发送将INT引脚拉高，如果不成功则拉高后再拉低重试发送
  * 
  * 主机<->从机(双工通信)：
  * 当从机信息实时性要求不高的时候(电池数据等周期大于1ms的)，可以先将信息存入BUFFER
    但是不拉低INT引脚
  * 当然需要先检查CSS位是否拉高(是否允许传输)
  * 等待主机主动与从机通信的时候再传输
  * 
  * 规则:CSS拉高的时候可以允许从机发送
  *     INT拉高的时候可以允许主机发送
  *     INT拉低的时候主机不发送
  */

InterboardState interboardTxState = INTERBOARD_READY;
InterboardState interboardRxState = INTERBOARD_READY;
static uint8_t  interboardRxBuf[55];
static uint8_t  interboardTxBuf[2][55];
static uint8_t  ackRx;
static uint8_t  ackTx;
static uint8_t  txBufUsing;
static uint8_t  txBufPending;

//MASTER 实现逻辑
#ifdef MASTER_BOARD

#define NSS_H() HAL_GPIO_WritePin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin, GPIO_PIN_SET)
#define NSS_L() HAL_GPIO_WritePin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin, GPIO_PIN_RESET)
#define READ_INT() HAL_GPIO_ReadPin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin)

void Interboard_init() {
  NSS_H();
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR0_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR1_Pin, GPIO_PIN_SET);
  while (READ_INT() != GPIO_PIN_SET)
    ;

  interboardTxState = INTERBOARD_READY;
  interboardRxState = INTERBOARD_READY;
  return;
}

/**
  * @brief    板间通信主机发送接收完成回调
  * 
  */
void Interboard_txRxCpltHook(void) {
  //判断是否接收成功
  for (uint8_t i; i < sizeof(interboardRxBuf); i++) {
    if (interboardRxBuf[i] = INTERBOARD_FRAME_HEAD) {
      switch (interboardRxBuf[i + 1]) {
      case INTERBOARDMSG_CAN: ackTx = InterBoard_canRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_BATT: ackTx = InterBoard_battRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_IMU: ackTx = InterBoard_imuRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_COMPUTER: ackTx = InterBoard_computerRxHook(interboardRxBuf[i + 2]); break;
      default: ackTx = 0;
      }
      if (ackTx == INTERBOARD_FRAME_ACK) {
        break;
      }
    }
  }
  //等待一下从机准备好BUFFER
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  //发送ACK信号
  HAL_SPI_TransmitReceive(&HSPI, &ackTx, &ackRx, 1, 1);
  NSS_H();
  if (ackRx != INTERBOARD_FRAME_ACK && interboardTxState != INTERBOARD_READY) {
    //发送失败重传
    __NOP();
    __NOP();
    __NOP()
    NSS_L();
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
    return;
  } else if (txBufUsing != txBufPending) {
    //继续发送下一帧
    txBufUsing = txBufPending;
    __NOP();
    __NOP();
    __NOP();
    NSS_L();
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
    --interboardTxState;
    return;
  } else {
    //发送完成
    if (interboardTxState != INTERBOARD_READY) {
      --interboardTxState;
    } else if (interboardRxState != INTERBOARD_READY) {
      interboardRxState = INTERBOARD_READY;
    }
    return;
  }
}

/**
  * @brief    板间通信从机请求发送回调
  * 
  */
void Interboard_rxReqHook() {
  if (interboardRxState == INTERBOARD_READY) {
    interboardRxState = INTERBOARD_BUSY;
    interboardTxState++;
    memset(&interboardTxBuf[txBufUsing], 0, 55);
    NSS_L();
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
  }
}

uint8_t Interboard_tx(InterboardMsgType msgType, uint8_t len, uint8_t *buf) {
  if (interboardTxState < INTERBOARD_BUSY) {
    txBufPending ^= 1;
    interboardTxBuf[txBufPending][0] = INTERBOARD_FRAME_HEAD;
    interboardTxBuf[txBufPending][1] = msgType;
    interboardTxBuf[txBufPending][2] = len;
    memcpy(interboardTxBuf[txBufPending] + 3, buf, len);
    if (interboardTxState < INTERBOARD_PENDING) {
      txBufUsing = txBufPending;
      NSS_L();
      HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
    }
    ++interboardTxState;
  } else {
    return 0;
  }
}

#elif defined SLAVE_BOARD

#define INT_H() HAL_GPIO_WritePin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin, GPIO_PIN_SET)
#define INT_L() HAL_GPIO_WritePin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin, GPIO_PIN_RESET)
#define READ_NSS() HAL_GPIO_ReadPin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin)

void Interboard_init() {
  INT_H();
  while (READ_NSS() != GPIO_PIN_SET)
    ;
  interboardTxState = INTERBOARD_READY;
  interboardRxState = INTERBOARD_READY;
  return;
}

/**
  * @brief    板间通信从机发送接收完成回调
  * 
  */
void Interboard_txRxCpltHook(void) {
  //判断是否接收成功
  for (uint8_t i; i < sizeof(interboardRxBuf); i++) {
    if (interboardRxBuf[i] = INTERBOARD_FRAME_HEAD) {
      switch (interboardRxBuf[i + 1]) {
      case INTERBOARDMSG_CAN: ackTx = InterBoard_canRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_BATT: ackTx = InterBoard_battRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_IMU: ackTx = InterBoard_imuRxHook(interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_COMPUTER: ackTx = InterBoard_computerRxHook(interboardRxBuf[i + 2]); break;
      default: ackTx = 0;
      }
      if (ackTx == INTERBOARD_FRAME_ACK) {
        break;
      }
    }
  }
  //发送ACK信号
  HAL_SPI_TransmitReceive(&HSPI, &ackTx, &ackRx, 1, 1);
  NSS_H();
  if (ackRx != INTERBOARD_FRAME_ACK && interboardTxState != INTERBOARD_READY) {
    //发送失败重传
    __NOP();
    __NOP();
    __NOP()
    NSS_L();
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
    return;
  } else if (txBufUsing != txBufPending) {
    //继续发送下一帧
    txBufUsing = txBufPending;
    __NOP();
    __NOP();
    __NOP();
    NSS_L();
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
    --interboardTxState;
    return;
  } else {
    //发送完成
    if (interboardTxState != INTERBOARD_READY) {
      --interboardTxState;
    } else if (interboardRxState != INTERBOARD_READY) {
      interboardRxState = INTERBOARD_READY;
    }
    return;
  }
}

uint8_t Interboard_tx(InterboardMsgType msgType, uint8_t len, uint8_t *buf) {
  if (interboardTxState < INTERBOARD_BUSY) {
    txBufPending ^= 1;
    interboardTxBuf[txBufPending][0] = INTERBOARD_FRAME_HEAD;
    interboardTxBuf[txBufPending][1] = msgType;
    interboardTxBuf[txBufPending][2] = len;
    memcpy(interboardTxBuf[txBufPending] + 3, buf, len);
    if (interboardTxState < INTERBOARD_PENDING) {
      txBufUsing = txBufPending;
      HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, 50);
      INT_L();
    }
    ++interboardTxState;
  } else {
    return 0;
  }
}

#endif
