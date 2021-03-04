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

extern void RM_Motor_RxHook(CAN_Frame* frame);
extern void RMD_Motor_RxHook(CAN_Frame* frame);
extern void VESC_Motor_RxHook(CAN_Frame* frame);

void CAN_RxHook(CAN_Frame* frame) {
  if(frame->type==CAN_FRAME_STD){
    switch(frame->id){
    case ENCODER_RX: break; //Encoder_RxHook(frame); break;
    case RM_MOTOR_RX_L ... RM_MOTOR_RX_H: RM_Motor_RxHook(frame); break;
    case RMD_MOTOR_RX_L ... RMD_MOTOR_RX_H: RMD_Motor_RxHook(frame); break;
    default: break;
    }
  }else if(frame->type==CAN_FRAME_EXT){
    VESC_Motor_RxHook(frame);
  }
}

void CAN_Tx(CAN_Frame *frame) { 
  static CAN_TxHeaderTypeDef TxHeader;
  TxHeader.DLC = frame->len;
  TxHeader.IDE = frame->type == CAN_FRAME_STD ? CAN_ID_STD : CAN_ID_EXT;
  TxHeader.StdId = frame->type == CAN_FRAME_STD ? frame->id : 0;
  TxHeader.ExtId = frame->type == CAN_FRAME_STD ? 0 : frame->id;
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime = DISABLE;
  HAL_CAN_AddTxMessage(frame->device == INTERNAL_CAN1 ? &hcan1 : &hcan2, &TxHeader, frame->data, NULL);
}
