/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     can_comm.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    CAN统一收发函数，后期可拓展支持外置CAN收发器
  * @version  0.1
  * @date     2021-02-20
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "can_comm.h"
#include "interboard_spi.h"
#include "string.h"
#include "small_queue.h"

static uint8_t             canInterboardBuf[sizeof(CAN_Frame) + 8];
static CAN_TxHeaderTypeDef canTxHeaderBuf[2][3];
static uint8_t             canTxDataBuf[2][3][8];
static SmallQueue          canTxQueue[2];

extern void RM_Motor_rxHook(CAN_Frame *frame);
extern void RMD_Motor_RxHook(CAN_Frame *frame);
extern void VESC_Motor_RxHook(CAN_Frame *frame);
extern void Encoder_RxHook(CAN_Frame *frame);

void CAN_TxCpltHook(CAN_HandleTypeDef *hcan) {
  uint8_t i = 255;
  if (hcan == &hcan1) {
    i = 0;
  } else if (hcan == &hcan2) {
    i = 1;
  }
  if (!SmallQueue_isEmpty(&canTxQueue[i])) {
    HAL_CAN_AddTxMessage(hcan, &canTxHeaderBuf[i][SmallQueue_getDequeueIndex(&canTxQueue[i])],
                         canTxDataBuf[i][SmallQueue_getDequeueIndex(&canTxQueue[i])], NULL);
    SmallQueue_dequeue(&canTxQueue[i]);
  }
}

void CAN_RxHook(CAN_Frame *frame) {
  if (frame->type == CAN_FRAME_STD) {
    switch (frame->id) {
    case ENCODER_RX_L ... ENCODER_RX_H: Encoder_RxHook(frame); break;
    case RM_MOTOR_RX_L ... RM_MOTOR_RX_H: RM_Motor_rxHook(frame); break;
    case RMD_MOTOR_RX_L ... RMD_MOTOR_RX_H: RMD_Motor_RxHook(frame); break;
    default: break;
    }
  } else if (frame->type == CAN_FRAME_EXT) {
    VESC_Motor_RxHook(frame);
  }
}

void CAN_Tx(CAN_Frame *frame) {
  if (frame->device <= INTERNAL_CAN2) {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC                = frame->len;
    TxHeader.IDE                = frame->type == CAN_FRAME_STD ? CAN_ID_STD : CAN_ID_EXT;
    TxHeader.StdId              = frame->type == CAN_FRAME_STD ? frame->id : 0;
    TxHeader.ExtId              = frame->type == CAN_FRAME_STD ? 0 : frame->id;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;
    // while (0 == HAL_CAN_GetTxMailboxesFreeLevel(frame->device == INTERNAL_CAN1 ? &hcan1 : &hcan2))
    //   ;
    if (0 != HAL_CAN_GetTxMailboxesFreeLevel(frame->device == INTERNAL_CAN1 ? &hcan1 : &hcan2)){
      HAL_CAN_AddTxMessage(frame->device == INTERNAL_CAN1 ? &hcan1 : &hcan2, &TxHeader, frame->data, NULL);
    }else{
      //加入外置发送队列
      memcpy(&canTxHeaderBuf[frame->device][SmallQueue_getEnqueueIndex(&canTxQueue[frame->device])], &TxHeader,
             sizeof(CAN_TxHeaderTypeDef));
      memcpy(canTxDataBuf[frame->device][SmallQueue_getEnqueueIndex(&canTxQueue[frame->device])], frame->data, 8);
      SmallQueue_enqueue(&canTxQueue[frame->device]);
    }
  } else if (frame->device <= EXTERNAL_CAN2) {
    memset(canInterboardBuf, 0, sizeof(canInterboardBuf));
    memcpy(canInterboardBuf, frame, sizeof(CAN_Frame));
    memcpy(&canInterboardBuf[sizeof(CAN_Frame)], frame->data, frame->len);
    if (interboardTxState != INTERBOARD_BUSY) {
      Interboard_tx(INTERBOARDMSG_CAN, sizeof(canInterboardBuf), canInterboardBuf);
    }
  }
}

static void CAN_Filter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef CAN_FilterConfig;

  //WARN: CAN2的FilterBank必须大于等于14才能使能接收中断!!!!!
  CAN_FilterConfig.FilterBank  = hcan == &hcan1 ? 0 : 14;
  CAN_FilterConfig.FilterMode  = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  CAN_FilterConfig.FilterIdHigh = 0;
  CAN_FilterConfig.FilterIdLow  = 0;

  CAN_FilterConfig.FilterMaskIdHigh     = 0;
  CAN_FilterConfig.FilterMaskIdLow      = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  CAN_FilterConfig.FilterActivation     = ENABLE;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfig) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Start(CAN_Device device) {
  if (device <= INTERNAL_CAN2) {
    CAN_HandleTypeDef *hcan = device == INTERNAL_CAN1 ? &hcan1 : &hcan2;
    CAN_Filter_Init(hcan);
    SmallQueue_init(&canTxQueue[device], 3);
    if (HAL_CAN_Start(hcan) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
      Error_Handler();
    }
  }
}
