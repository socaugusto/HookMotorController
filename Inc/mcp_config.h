
/**
  ******************************************************************************
  * @file    mcp_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration definition of the MCP protocol
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef MCP_CONFIG_H
#define MCP_CONFIG_H

#include "mcp.h"
#include "aspep.h"
#include "stlink_mcptl.h"

#define USARTA USART1
#define DMA_RX_A DMA1
#define DMA_TX_A DMA1
#define DMACH_RX_A LL_DMA_CHANNEL_3
#define DMACH_TX_A LL_DMA_CHANNEL_2
#define USARTA_IRQHandler USART1_IRQHandler
#define MCP_RX_IRQHandler_A DMA1_Channel2_3_IRQHandler

extern ASPEP_Handle_t aspepOverUartA;
extern MCP_Handle_t MCP_Over_UartA;
extern STLNK_Handle_t STLNK;
extern MCP_Handle_t MCP_Over_STLNK;
extern STLNK_Control_t stlnkCtrl;
#endif /* MCP_CONFIG_H */

/************************ (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
