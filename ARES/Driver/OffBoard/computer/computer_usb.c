/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     computer_usb.c
  * @author   ljy
  * @brief    计算机通信
  * @version  0.1
  * @date     2021-07-24
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "computer_usb.h"

static uint32_t usbBufLen = 0;
static uint8_t  usbBuf[100];

uint8_t Interboard_computerRxCpltHook(uint8_t *buf, uint32_t len) {

#ifdef MASTER_BOARD
  // A板发给C板
  //InterboardUsb_tx(buf, len);
#elif defined SLAVE_BOARD
  // C板发回给电脑
  CDC_Transmit_FS(buf, len);
#endif

  return 1;
}

uint8_t Interboard_computerRxHook(uint8_t *buf, uint8_t len) {

  if (buf[0] & (0x7F) && (buf[0] & (0x7F)) < COMPUTER_MAX_PACKET_NUM) {
    memcpy(usbBuf + (buf[0] & (0x7F)) * COMPUTER_PACKET_MAX_SIZE, buf + 1, len);

    // 判断是否为最后一个包
    if (READ_BIT(buf[0], 1 << 7) == 1 << 7) {
      usbBufLen = (buf[0] & (0x7F)) * COMPUTER_PACKET_MAX_SIZE + len;
      Interboard_computerRxCpltHook(usbBuf, usbBufLen);
    }
    return 1;
  } else {
    return 0;
  }
}

#ifdef MASTER_BOARD
void Computer_tx(uint8_t *Buf, uint32_t Len){
#elif defined SLAVE_BOARD
void Computer_rxHook(uint8_t *Buf, uint32_t Len) {
#endif
    /*
    第一位后7Bit表示包序号
    第一位第8Bit表示是否为最后一个包
    其它COMPUTER_PACKET_MAX_SIZE位表示数据
  */
    uint32_t bufNum = (Len - 1) / COMPUTER_PACKET_MAX_SIZE + 1;
  // 初始化
  uint8_t buffer[COMPUTER_PACKET_MAX_SIZE + 1];
  for (int i = 0; i < 21; i++)
    buffer[i] = 0;

  // 发送
  for (int i = 0; i < bufNum; i++) {

    buffer[0] = i;
    if (i == bufNum - 1) {
      SET_BIT(buffer[0], 1 << 7);
      memcpy(buffer + 1, Buf + i * COMPUTER_PACKET_MAX_SIZE, Len - (bufNum - 1) * COMPUTER_PACKET_MAX_SIZE);

      while (interboardTxState != INTERBOARD_READY)
        ;
      Interboard_tx(INTERBOARDMSG_COMPUTER, Len - (bufNum - 1) * COMPUTER_PACKET_MAX_SIZE + 1, buffer);
    } else {
      memcpy(buffer + 1, Buf + i * COMPUTER_PACKET_MAX_SIZE, COMPUTER_PACKET_MAX_SIZE);
      while (interboardTxState != INTERBOARD_READY)
        ;
      Interboard_tx(INTERBOARDMSG_COMPUTER, COMPUTER_PACKET_MAX_SIZE + 1, buffer);
    }
  }
}
