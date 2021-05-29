/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mcp2515.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    MCP2515驱动程序
  * @version  0.1
  * @date     2021-04-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "mcp2515.h"
#include "mcp2515_reg.h"
#include "spi.h"
#include "main.h"
#include "monitor_task.h"
#include <string.h>
#include "cmsis_os.h"

#define MCP_NSS_H() HAL_GPIO_WritePin(MCP_NSS_GPIO_Port, MCP_NSS_Pin, GPIO_PIN_SET)
#define MCP_NSS_L() HAL_GPIO_WritePin(MCP_NSS_GPIO_Port, MCP_NSS_Pin, GPIO_PIN_RESET)

typedef enum {
  MCP_RXBUF_RXSIDH_OFFSET = 1,
  MCP_RXBUF_RXSIDL_OFFSET,
  MCP_RXBUF_RXEID8_OFFSET,
  MCP_RXBUF_RXEID0_OFFSET,
  MCP_RXBUF_RXDLC_OFFSET,
  MCP_RXBUF_RXD0_OFFSET,
  MCP_RXBUF_RXD1_OFFSET,
  MCP_RXBUF_RXD2_OFFSET,
  MCP_RXBUF_RXD3_OFFSET,
  MCP_RXBUF_RXD4_OFFSET,
  MCP_RXBUF_RXD5_OFFSET,
  MCP_RXBUF_RXD6_OFFSET,
  MCP_RXBUF_RXD7_OFFSET,
  MCP_RXBUF_CANSTAT_OFFSET,
} Mcp2515RxBufOffset;

typedef enum {
  MCP_TXBUF_COMMAND_OFFSET = 0,
  MCP_TXBUF_TXSIDH_OFFSET,
  MCP_TXBUF_TXSIDL_OFFSET,
  MCP_TXBUF_TXEID8_OFFSET,
  MCP_TXBUF_TXEID0_OFFSET,
  MCP_TXBUF_TXDLC_OFFSET,
  MCP_TXBUF_TXD0_OFFSET,
  MCP_TXBUF_TXD1_OFFSET,
  MCP_TXBUF_TXD2_OFFSET,
  MCP_TXBUF_TXD3_OFFSET,
  MCP_TXBUF_TXD4_OFFSET,
  MCP_TXBUF_TXD5_OFFSET,
  MCP_TXBUF_TXD6_OFFSET,
  MCP_TXBUF_TXD7_OFFSET,
  MCP_TXBUF_CANSTAT_OFFSET,
} Mcp2515TxBufOffset;

static uint8_t     txBuf[18];
static uint8_t     rxBuf[18];
static uint8_t     errorFlag;
static CAN_Frame   rxFrame;
static MonitorList mcpMonitor;
static uint32_t    receiveTime;

static uint8_t readReg  = EFLG;
static uint8_t data8[8] = {0};

static void Mcp2515_guard(uint32_t *errorReg, uint32_t *errorTime);

/**
  * @brief    MCP2515硬件初始化指令
  * 
  */
static void Mcp2515_hardReset(void) {
  uint8_t reset = CAN_RESET;

  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  MCP_NSS_L();
  HAL_SPI_Transmit(&hspi1, &reset, 1, 1);
  MCP_NSS_H();
}

/**
  * @brief    读MCP2515寄存器
  * 
  * @param    reg       寄存器地址
  * @return   uint8_t   寄存器值
  */
static uint8_t Mcp2515_readReg(uint8_t reg) {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_READ;
  txBuf[1] = reg;

  MCP_NSS_L();
  HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 3, 1);
  MCP_NSS_H();
  return rxBuf[2];
}

/**
  * @brief    MCP2515读寄存器组
  * 
  * @param    reg       寄存器组开始地址
  * @param    len       寄存器组长度
  * 
  * @note     寄存器值保存在RxBuf中
  */
static void Mcp2515_readRegs(uint8_t reg, uint8_t len) {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_READ;
  txBuf[1] = reg;

  MCP_NSS_L();
  HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, len + 2, 1);
  MCP_NSS_H();
}

/**
  * @brief    写MCP2515寄存器
  * 
  * @param    reg       寄存器地址
  * @param    value     寄存器值
  */
static void Mcp2515_writeReg(uint8_t reg, uint8_t value) {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_WRITE;
  txBuf[1] = reg;
  txBuf[2] = value;

  MCP_NSS_L();
  HAL_SPI_Transmit(&hspi1, txBuf, 3, 1);
  MCP_NSS_H();
}

/**
  * @brief    写MCP2515寄存器组
  * 
  * @param    reg       寄存器地址
  * @param    buf       待写的寄存器值
  * @param    len       寄存器组长度
  * 
  * @note     如果BUFF为NULL，则清空寄存器组
  */
static void Mcp2515_writeRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  if (buf) {
    txBuf[0] = CAN_WRITE;
    txBuf[1] = reg;
    memcpy(txBuf + 2, buf, len);
  } else {
    memset(txBuf, 0, sizeof(txBuf));
    txBuf[0] = CAN_WRITE;
    txBuf[1] = reg;
  }

  MCP_NSS_L();
  HAL_SPI_Transmit(&hspi1, txBuf, 3, 1);
  MCP_NSS_H();
}

/**
  * @brief    修改MCP2515某个寄存器中某bit
  * 
  * @param    reg       寄存器
  * @param    mask      掩码
  * @param    value     值
  */
static void Mcp2515_modifyReg(uint8_t reg, uint8_t mask, uint8_t value) {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_BIT_MODIFY;
  txBuf[1] = reg;
  txBuf[2] = mask;
  txBuf[3] = value;

  HAL_SPI_Transmit(&hspi1, txBuf, 4, 1);
}

/**
  * @brief    获得接收状态标志寄存器值
  * 
  * @return   uint8_t   接收状态标志寄存器值
  */
static uint8_t Mcp2515_getRxStatus() {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_RX_STATUS;

  MCP_NSS_L();
  HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, 1);
  MCP_NSS_H();

  return rxBuf[1];
}

/**
  * @brief    获取MCP2515的状态
  * 
  */
static uint8_t Mcp2515_getStatus() {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_RD_STATUS;

  MCP_NSS_L();
  HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, 1);
  MCP_NSS_H();

  return rxBuf[1];
}

/**
  * @brief    直接读取RX寄存器指令 从RXB0SIDH开始一直读取到CANSTAT
  * 
  */
static void Mcp2515_readRx() {
  //while (hspi1.State != HAL_SPI_STATE_READY) {}

  txBuf[0] = CAN_RD_RX_BUFF;

  MCP_NSS_L();
  HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 15, 1);
  MCP_NSS_H();
}

/**
  * @brief    读取EFLG的值
  * 
  * @return   uint8_t   EFLG的值
  */
static uint8_t Mcp2515_getErrStatus() { return Mcp2515_readReg(EFLG); }

/**
  * @brief    MCP2515初始化 初始化监视项目和初始化MCP2515寄存器
  * 
  */
void Mcp2515_init(void) {
  Mcp2515_hardReset();
  HAL_Delay(10);

  //清除过滤器
  Mcp2515_writeReg(RXM0SIDH, 0);
  Mcp2515_writeReg(RXM0SIDH, 0);
  Mcp2515_writeReg(RXF0SIDL, 0);
  Mcp2515_writeReg(RXF0SIDL, 0);

  Mcp2515_writeRegs(TXB0CTRL, NULL, 14);
  Mcp2515_writeRegs(TXB1CTRL, NULL, 14);
  Mcp2515_writeRegs(TXB2CTRL, NULL, 14);

  Mcp2515_writeReg(RXB0CTRL, 0x00);
  Mcp2515_writeReg(RXB1CTRL, 0x00);
  Mcp2515_writeReg(TXB0CTRL, TXP_HIGHEST);

  //在16M晶振下设置总线波特率为1M
  //参考https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can_dfs.h
  Mcp2515_writeReg(CNF1, 0x00); 
  Mcp2515_writeReg(CNF2, 0x91);
  Mcp2515_writeReg(CNF3, 0x01);

  //设置中断条件
  Mcp2515_writeReg(CANINTE, RX0IE | RX1IE | ERRIE);
  Mcp2515_writeReg(BFPCTRL, 0x00);
  Mcp2515_writeReg(TXRTSCTRL, 0x00);

  //启动CAN
  Mcp2515_writeReg(CANCTRL, OPMODE_NORMAL);

  Monitor_registor(&mcpMonitor, Mcp2515_guard, MONITOR_MCP_ID);

  Mcp2515_writeReg(CANINTF, 0);
}

/**
  * @brief    重设MCP2515
  * 
  */
static void Mcp2515_reset(void) {
  Mcp2515_readReg(readReg);
  Mcp2515_hardReset();
  vTaskDelay(5);
  Mcp2515_writeReg(CANCTRL, OPMODE_CONFIG);

  //清除过滤器
  Mcp2515_writeReg(RXM0SIDH, 0);
  Mcp2515_writeReg(RXM0SIDH, 0);
  Mcp2515_writeReg(RXF0SIDL, 0);
  Mcp2515_writeReg(RXF0SIDL, 0);

  Mcp2515_writeRegs(TXB0CTRL, NULL, 14);
  Mcp2515_writeRegs(TXB1CTRL, NULL, 14);
  Mcp2515_writeRegs(TXB2CTRL, NULL, 14);

  Mcp2515_writeReg(RXB0CTRL, 0x00);
  Mcp2515_writeReg(RXB1CTRL, 0x00);
  Mcp2515_writeReg(TXB0CTRL, TXP_HIGHEST);

  //在16M晶振下设置总线波特率为1M
  //参考https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can_dfs.h
  Mcp2515_writeReg(CNF1, 0x00);
  Mcp2515_writeReg(CNF2, 0x91);
  Mcp2515_writeReg(CNF3, 0x01);

  //设置中断条件
  Mcp2515_writeReg(CANINTE, RX0IE | RX1IE | ERRIE | MERRF);
  Mcp2515_writeReg(BFPCTRL, 0x00);
  Mcp2515_writeReg(TXRTSCTRL, 0x00);

  //启动CAN
  Mcp2515_writeReg(CANCTRL, OPMODE_NORMAL);
  Mcp2515_writeReg(CANINTF, 0);
}

/**
  * @brief    对收到的缓冲区解析成CAN帧
  * 
  */
static void Mcp2515_prase() {
  //读取CANSTAT位
  if (rxBuf[MCP_RXBUF_CANSTAT_OFFSET] == 0) {
    errorFlag = 0;
    if (!READ_BIT(rxBuf[MCP_RXBUF_RXSIDL_OFFSET], (1 << 3))) {
      //说明是标准帧
      memset(&rxFrame, 0, sizeof(CAN_Frame));
      rxFrame.type   = CAN_FRAME_STD;
      rxFrame.device = EXTERNAL_MCP2515;
      rxFrame.len    = rxBuf[MCP_RXBUF_RXDLC_OFFSET] & 0x0F;
      memcpy(&data8, &rxBuf[MCP_RXBUF_RXD0_OFFSET],8);
      rxFrame.data   = data8;
      rxFrame.id     = (rxBuf[MCP_RXBUF_RXSIDH_OFFSET] << 3) | (rxBuf[MCP_RXBUF_RXSIDL_OFFSET] >> 5);
      CAN_RxHook(&rxFrame);
    } else {
      //说明是扩展帧
      memset(&rxFrame, 0, sizeof(CAN_Frame));
      rxFrame.type   = CAN_FRAME_EXT;
      rxFrame.device = EXTERNAL_MCP2515;
      rxFrame.len    = rxBuf[MCP_RXBUF_RXDLC_OFFSET] & 0x0F;
      rxFrame.data   = &rxBuf[MCP_RXBUF_RXD0_OFFSET];
      rxFrame.id     = (rxBuf[MCP_RXBUF_RXEID8_OFFSET] << 8) | rxBuf[MCP_RXBUF_RXEID0_OFFSET] |
                   ((rxBuf[MCP_RXBUF_RXSIDL_OFFSET] & 0x03) << 16);
      CAN_RxHook(&rxFrame);
    }
  } else if ((rxBuf[MCP_RXBUF_CANSTAT_OFFSET] & 0x07) == 0x01) {
    //出错
    errorFlag = 1;
  }
}

/**
  * @brief    接收钩子
  * 
  */
void Mcp2515_rxHook(void) {
 
  Mcp2515_readRx();
  Mcp2515_prase();
  receiveTime = xTaskGetTickCount();
  
   Mcp2515_writeReg(CANINTF, 0);
}

/**
  * @brief    发送CAN帧的封装
  * 
  * @param    frame     CAN帧结构体
  */
void Mcp2515_tx(CAN_Frame *frame) {
  if (frame->device != EXTERNAL_MCP2515) {
    return;
  }
  while (hspi1.State != HAL_SPI_STATE_READY) {
    vTaskDelay(1);
  }

  memset(&txBuf, 0, sizeof(txBuf));
  txBuf[MCP_TXBUF_COMMAND_OFFSET] = CAN_LOAD_TX;
  if (frame->type == CAN_FRAME_STD) {
    txBuf[MCP_TXBUF_TXSIDH_OFFSET] = frame->id >> 3;
    txBuf[MCP_TXBUF_TXSIDL_OFFSET] = (frame->id & 0x07) << 5;
    txBuf[MCP_TXBUF_TXDLC_OFFSET]  = frame->len & 0x0F;
    memcpy(&txBuf[MCP_TXBUF_TXD0_OFFSET], frame->data, frame->len);
  } else if (frame->type == CAN_FRAME_STD) {
    txBuf[MCP_TXBUF_TXSIDL_OFFSET] = 1 << 3 | (0x03 & (frame->id >> 16));
    txBuf[MCP_TXBUF_TXEID8_OFFSET] = 0xFF & (frame->id >> 8);
    txBuf[MCP_TXBUF_TXEID0_OFFSET] = 0xFF & (frame->id);
    txBuf[MCP_TXBUF_TXDLC_OFFSET]  = frame->len & 0x0F;
    memcpy(&txBuf[MCP_TXBUF_TXD0_OFFSET], frame->data, frame->len);
  }
  MCP_NSS_L();
  HAL_SPI_Transmit(&hspi1, txBuf, sizeof(txBuf), 1);
  MCP_NSS_H();
  __NOP(); //增加一些延迟 使得NSS有时间上拉
  txBuf[0] = CAN_RTS;
  MCP_NSS_L();
  HAL_SPI_Transmit(&hspi1, txBuf, 1, 1);
  MCP_NSS_H();
}

static uint32_t debug = 0;
/**
  * @brief    MCP2515守护函数
  * 
  * @param    errorReg  错误寄存器
  * @param    errorTime 错误时间
  */
static void Mcp2515_guard(uint32_t *errorReg, uint32_t *errorTime) {
  if (xTaskGetTickCount() - receiveTime > 200) {
    debug = xTaskGetTickCount() - receiveTime;

    
      SET_BIT(*errorReg, MONITOR_ERROR_LOST);
      if (!READ_BIT(*errorReg, MONITOR_ERROR_EXIST)) {
        *errorTime = receiveTime;
      }
    
    
  } else {
    CLEAR_BIT(*errorReg, MONITOR_ERROR_LOST);
  }
  if (errorFlag != 0x40 && errorFlag != 0) {
    SET_BIT(*errorReg, MONITOR_ERROR_INVALID);
    if (!READ_BIT(*errorReg, MONITOR_ERROR_EXIST)) {
      *errorTime = receiveTime;
    }
  } else {
    CLEAR_BIT(*errorReg, MONITOR_ERROR_INVALID);
  }
  if ((*errorReg & 0x7FFFFFFF) == 0) {
    *errorReg  = 0;
    *errorTime = 0;
  } else {
    SET_BIT(*errorReg, MONITOR_ERROR_EXIST);
    if (((xTaskGetTickCount() - *errorTime) > MCP_GET_ERR_TIMEOUT) &&
        ((xTaskGetTickCount() - *errorTime) < MCP_RESET_TIMEOUT)) {
      errorFlag = Mcp2515_getErrStatus();
      *errorReg |= (uint32_t)errorFlag;
      Mcp2515_writeReg(CANINTF, 0);
      Mcp2515_readReg(CANINTF);
    } else if ((xTaskGetTickCount() - *errorTime) > MCP_RESET_TIMEOUT) {
      *errorTime = xTaskGetTickCount();
      *errorReg  = 0;
      errorFlag  = 0;
      Mcp2515_writeReg(CANINTF, 0);
      Mcp2515_reset();
    }
  }
}
