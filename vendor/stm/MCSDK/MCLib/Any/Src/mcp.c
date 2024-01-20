/**
  ******************************************************************************
  * @file    mcp.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the MCP protocol
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

#include "stdint.h"
#include "mcp.h"
#include "register_interface.h"
#include "mc_config.h"

void MCP_ReceivedPacket(MCP_Handle_t * pHandle)
{
  uint16_t * packetHeader;
  uint16_t command;
  uint8_t motorID;
  uint8_t MCPResponse;
  
  if (pHandle->rxLength != 0)
  {
    packetHeader = (uint16_t *) pHandle->rxBuffer;  
    command = *packetHeader & CMD_MASK;
    motorID = (*packetHeader & MOTOR_MASK)-1;
    
    MCI_Handle_t * pMCI = &Mci[motorID];
    /* Initialization of the tx length, command which send back data has to increment the txLength (case of Read register)*/
    pHandle->txLength = 0;
    
    switch (command) 
      {
      case SET_DATA_ELEMENT:
        MCPResponse = RI_SetRegCommandParser (pHandle); 
      break;
      case GET_DATA_ELEMENT:
        MCPResponse = RI_GetRegCommandParser (pHandle); 
        break;
      case START_MOTOR:
        MCPResponse = (MCI_StartMotor(pMCI)) ? MCP_CMD_OK : MCP_CMD_NOK;
        break;
      case STOP_MOTOR: /* Todo: Check the pertinance of return value*/
        MCI_StopMotor(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      case STOP_RAMP:
        {
          if (MCI_GetSTMState(pMCI) == RUN)
          {
            MCI_StopRamp(pMCI);
          }
        MCPResponse = MCP_CMD_OK;
        }
        break;
      case START_STOP:
        {
          /* Queries the STM and a command start or stop depending on the state. */
          if (MCI_GetSTMState(pMCI) == IDLE)
          {
            MCPResponse = (MCI_StartMotor(pMCI)) ? MCP_CMD_OK : MCP_CMD_NOK;
          }
          else
          {
            MCI_StopMotor(pMCI);
            MCPResponse = MCP_CMD_OK;
          }
        }
        break;
      case FAULT_ACK:
        {
          MCI_FaultAcknowledged(pMCI);
          MCPResponse = MCP_CMD_OK;
        }
        break;
      case ENCODER_ALIGN:
        MCI_EncoderAlign(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      case IQDREF_CLEAR:
        MCI_Clear_Iqdref(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      case PFC_ENABLE:
      case PFC_DISABLE:
      case PFC_FAULT_ACK:
      default :
        MCPResponse = MCP_CMD_UNKNOWN;
      }
      pHandle->txBuffer[pHandle->txLength] = MCPResponse;
      pHandle->txLength++;
  }
  else /* Length is 0, this is a request to send back the last packet */
  {
    /* Nothing to do, txBuffer and txLength have not been modified */
  } 
}

/******************* (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
