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
#include "mcp2515.h"

extern void RM_Motor_rxHook(CAN_Frame *frame);
extern void RMD_Motor_RxHook(CAN_Frame *frame);
extern void VESC_Motor_RxHook(CAN_Frame *frame);
extern void Encoder_RxHook(CAN_Frame *frame);

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
    HAL_CAN_AddTxMessage(frame->device == INTERNAL_CAN1 ? &hcan1 : &hcan2, &TxHeader, frame->data, NULL);
  } else if (frame->device == EXTERNAL_MCP2515) {
    Mcp2515_tx(frame);
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
    if (HAL_CAN_Start(hcan) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
    }
  } else if (device == EXTERNAL_MCP2515) {
    Mcp2515_init();
  }
}
