/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     interboard_spi.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    SPI板间通信协议
  * @version  0.1
  * @date     2021-06-26
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "interboard_spi.h"
#include "string.h"
#include "can_comm.h"
#ifdef MASTER_BOARD
#include "monitor_task.h"
#endif

#ifdef MASTER_BOARD
#define HSPI hspi1
#define NSS_H() HAL_GPIO_WritePin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin, GPIO_PIN_SET)
#define NSS_L() HAL_GPIO_WritePin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin, GPIO_PIN_RESET)
#define READ_INT() HAL_GPIO_ReadPin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin)
static void   Interboard_guard(uint32_t *errorReg, uint32_t *errorTime);
MonitorList interboardMonitor;
#elif defined SLAVE_BOARD
#define HSPI hspi2
#define INT_H() HAL_GPIO_WritePin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin, GPIO_PIN_SET)
#define INT_L() HAL_GPIO_WritePin(INTERBOARD_INT_GPIO_Port, INTERBOARD_INT_Pin, GPIO_PIN_RESET)
#define READ_NSS() HAL_GPIO_ReadPin(INTERBOARD_NSS_GPIO_Port, INTERBOARD_NSS_Pin)
#else
#error Please define board role!
#endif

fp32 batteryVoltage;



static fp32     interboardFrequency;
static uint32_t count;
static uint32_t lastTime;

static uint8_t txBufUsing;

static uint8_t interboardRxBuf[INTERBOARD_MAX_FRAME_LENGTH];
static uint8_t interboardTxBuf[INTERBOARD_TXBUF_SIZE][INTERBOARD_MAX_FRAME_LENGTH];

InterboardState interboardTxState = INTERBOARD_READY;
InterboardState interboardRxState = INTERBOARD_READY;

static uint8_t Interboard_canRxHook(uint8_t *buf) {
  CAN_Frame frame;
  if (buf[0] == sizeof(CAN_Frame) + 8) {
    memcpy(&frame, &buf[1], sizeof(CAN_Frame));
    frame.data = &buf[sizeof(CAN_Frame) + 1];

#ifdef MASTER_BOARD
    CAN_RxHook(&frame);
#elif defined SLAVE_BOARD
    frame.device -= 2;
    CAN_Tx(&frame);
#endif

    return 1;
  } else {
    return 0;
  }
}

static uint8_t Interboard_battRxHook(uint8_t *buf) {
  if (buf[0] == sizeof(fp32)) {
    memcpy(&batteryVoltage, &buf[1], sizeof(fp32));
    return 1;
  } else {
    return 0;
  }
}

static uint8_t Interboard_imuRxHook(uint8_t *buf) { return 0; }

static uint8_t Interboard_computerRxHook(uint8_t *buf) { return 0; }

/**
  * @brief    使用错误处理函数 可以实现热插拔
  * 
  */
void Interboard_errorHook() {
  if (READ_BIT(HSPI.ErrorCode, HAL_SPI_ERROR_OVR)) {
    __HAL_SPI_CLEAR_OVRFLAG(&HSPI);
  }
  if (HAL_SPI_GetState(&HSPI) == HAL_SPI_STATE_READY) {
    HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, INTERBOARD_MAX_FRAME_LENGTH);
  }
}

/**
  * @brief    接收完成回调
  * 
  */
void Interboard_txRxCpltHook() {

  count++;
#ifdef MASTER_BOARD
  NSS_H();
#elif defined SLAVE_BOARD
  //闪烁指示灯
  if (HAL_GetTick() % 100 == 0) {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  }
#endif
  uint8_t txSuccess = 0;
  uint8_t rxSuccess = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (READ_BIT(interboardRxBuf[i], INTERBOARD_FRAME_ACK_BIT)) {
      txSuccess = 1;
    }
    if (interboardRxBuf[i] & INTERBOARD_FRAME_HEAD == INTERBOARD_FRAME_HEAD) {
      switch (interboardRxBuf[i + 1]) {
      case INTERBOARDMSG_CAN: rxSuccess = Interboard_canRxHook(&interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_BATT: rxSuccess = Interboard_battRxHook(&interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_IMU: rxSuccess = Interboard_imuRxHook(&interboardRxBuf[i + 2]); break;
      case INTERBOARDMSG_COMPUTER: rxSuccess = Interboard_computerRxHook(&interboardRxBuf[i + 2]); break;
      default: rxSuccess = 0; break;
      }
    }
    if (rxSuccess) {
      break;
    }
  }

  //如果发送成功才发送下一个缓冲区否则重发
  txBufUsing = (txBufUsing + txSuccess) % INTERBOARD_TXBUF_SIZE;
  if (interboardTxState == INTERBOARD_READY) {
    memset(interboardTxBuf[txBufUsing], 0, INTERBOARD_MAX_FRAME_LENGTH);
  } else {
    //如果发送成功才减少发送缓冲区的占用标志
    interboardTxState -= txSuccess;
  }

  //如果接收成功，发送ACK
  if (rxSuccess) {
    SET_BIT(interboardTxBuf[txBufUsing][0], INTERBOARD_FRAME_ACK_BIT);
  }

#ifdef MASTER_BOARD
  //等待一下从机准备好BUFFER
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  NSS_L();
  //NSS下拉需要时间
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
#endif

  __HAL_SPI_CLEAR_OVRFLAG(&HSPI);
  while (HAL_OK !=
         HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, INTERBOARD_MAX_FRAME_LENGTH)) {
    HAL_SPI_DMAStop(&HSPI);
  };
}

void Interboard_start() {
  memset(interboardTxBuf[txBufUsing], 0, INTERBOARD_MAX_FRAME_LENGTH);
  interboardRxState = INTERBOARD_BUSY;
  interboardTxState = INTERBOARD_READY;
#ifdef MASTER_BOARD
  NSS_L();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
#endif
  HAL_SPI_TransmitReceive_DMA(&HSPI, interboardTxBuf[txBufUsing], interboardRxBuf, INTERBOARD_MAX_FRAME_LENGTH);
}

void Interboard_init() {
#ifdef MASTER_BOARD
  NSS_H();
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR0_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR1_Pin, GPIO_PIN_RESET);
  while (READ_INT() != GPIO_PIN_SET)
    ;
  HAL_Delay(1);
  Monitor_registor(&interboardMonitor, Interboard_guard,-1);
#elif defined SLAVE_BOARD
  while (READ_NSS() != GPIO_PIN_SET)
    ;
  INT_H();
#endif
  interboardTxState = INTERBOARD_READY;
  interboardRxState = INTERBOARD_READY;

  Interboard_start();
  return;
}

void Interboard_tx(InterboardMsgType msgType, uint8_t len, uint8_t *buf) {
  if (interboardTxState != INTERBOARD_BUSY) {
    interboardTxState++;
    uint8_t buf2use             = (txBufUsing + interboardTxState) % INTERBOARD_TXBUF_SIZE;
    interboardTxBuf[buf2use][0] = INTERBOARD_FRAME_HEAD;
    interboardTxBuf[buf2use][1] = msgType;
    interboardTxBuf[buf2use][2] = len;
    memcpy(interboardTxBuf[buf2use] + 3, buf, len);
  }
}
#ifdef MASTER_BOARD
static void Interboard_restart(){
  interboardTxState = INTERBOARD_BUSY;
  interboardRxState = INTERBOARD_BUSY;
  
  NSS_H();
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR0_Pin, GPIO_PIN_SET);

  HAL_SPI_DMAStop(&HSPI);
  HAL_SPI_Abort(&HSPI);
  __HAL_SPI_CLEAR_FREFLAG(&HSPI);
  __HAL_SPI_CLEAR_OVRFLAG(&HSPI);
  __HAL_SPI_CLEAR_MODFFLAG(&HSPI);
  HAL_SPI_DeInit(&HSPI);
  vTaskDelay(10);
  HAL_SPI_Init(&HSPI);

  vTaskDelay(500);
  HAL_GPIO_WritePin(PWR0_GPIO_Port, PWR1_Pin, GPIO_PIN_RESET);
  while (READ_INT() != GPIO_PIN_SET){
    vTaskDelay(1);
  }
  vTaskDelay(1);
  

  Interboard_start();
  return;
}

static void Interboard_guard(uint32_t *errorReg, uint32_t *errorTime) {
  interboardFrequency = count * 100;
  count == 0;
  if (interboardFrequency <= 2000&&xTaskGetTickCount()>3000) {
    SET_BIT(*errorReg, MONITOR_ERROR_EXIST);
    SET_BIT(*errorReg, MONITOR_ERROR_LOST);
    *errorTime = xTaskGetTickCount();
  }else{
    *errorReg = 0;
    *errorTime = 0;
  }
  if(*errorReg!=0){
    Interboard_restart();
  }
}
#endif
