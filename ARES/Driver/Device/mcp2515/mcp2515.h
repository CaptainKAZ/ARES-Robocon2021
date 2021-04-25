/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mcp2515.h
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    MCP2515驱动程序
  * @version  0.1
  * @date     2021-04-14
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#ifndef MCP2515_H
#define MCP2515_H
#include "main.h"
#include "can_comm.h"

#define MCP_TIMEOUT 5
#define MCP_GET_ERR_TIMEOUT 60
#define MCP_RESET_TIMEOUT 1000

extern void Mcp2515_tx(CAN_Frame *frame);
extern void Mcp2515_rxHook(void);
extern void Mcp2515_init(void);

#endif
