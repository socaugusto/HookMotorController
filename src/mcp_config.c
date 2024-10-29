/**
  ******************************************************************************
  * @file    mcp_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration information of the MCP protocol
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

#include "parameters_conversion.h"
#include "usart_aspep_driver.h"
#include "aspep.h"
#include "stlink_mcptl.h"
#include "mcp.h"
#include "mcp_config.h"

uint8_t MCPSyncTxBuff[MCP_TX_SYNCBUFFER_SIZE];
uint8_t MCPSyncRXBuff[MCP_RX_SYNCBUFFER_SIZE];

UASPEP_Handle_t UASPEP_A =
{
 .USARTx = USARTA,
 .rxDMA = DMA_RX_A,
 .txDMA = DMA_TX_A,
 .rxChannel = DMACH_RX_A,
 .txChannel = DMACH_TX_A,
};

ASPEP_Handle_t aspepOverUartA =
{
  ._Super =
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .HWIp = &UASPEP_A,
  .Capabilities = {
    .DATA_CRC = 0,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX>>5)-1,
    .TXA_maxSize =  0,
    .version = 0x0,
  },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  .rxBuffer = &MCPSyncRXBuff[4],
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_receive = &UASPEP_RECEIVE_BUFFER,
  .fASPEP_send = &UASPEP_SEND_PACKET,
  .liid = 0,
};

MCP_Handle_t MCP_Over_UartA =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA,
};

STLNK_Handle_t STLNK =
{
  ._Super =
   {
     .fGetBuffer = &STLNK_getBuffer,
     .fSendPacket = &STLNK_sendPacket,
     .fRXPacketProcess = &STLNK_RXframeProcess,
   },
  .syncBuffer = {
   .buffer = MCPSyncTxBuff,
  },
  .rxBuffer = MCPSyncRXBuff,

};

MCP_Handle_t MCP_Over_STLNK =
{
  .pTransportLayer = (MCTL_Handle_t *) &STLNK,
};

//force Variable to section data in RAM -> variable is place has the beginning of the section and this is what we want

#if defined (__ICCARM__)
#pragma location=0x20000000
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".data")))
#endif
STLNK_Control_t stlnkCtrl =
{
  .rxBuffer = MCPSyncRXBuff,
  .syncBuffer = MCPSyncTxBuff,
  .asyncBufferA =  0,
  .asyncBufferB =  0,
};

/************************ (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
