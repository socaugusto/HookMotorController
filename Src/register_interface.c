/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
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
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mcp.h"
#include "mcp_config.h"
#include "mc_configuration_registers.h"

// [SILVIO] static uint16_t nullData16=0;

static uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t maxSize);
static uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t maxSize);
static uint8_t RI_MovString (const char * srcString, char * destString, uint16_t *size, uint16_t maxSize);

__weak uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer+MCP_HEADER_SIZE;
  uint8_t * txData = pHandle->txBuffer;
  int16_t rxLength = pHandle->rxLength-MCP_HEADER_SIZE;
  int16_t txSyncFreeSpace = pHandle->pTransportLayer->txSyncMaxPayload-1; /* 1 Byte must be available for MCP CMD status*/
  uint16_t size;
  uint8_t retVal=MCP_CMD_OK;
  uint8_t accessResult;
  uint8_t number_of_item =0;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     number_of_item ++;
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data
     accessResult = RI_SetReg (*dataElementID,rxData,&size,rxLength);

     /* Prepare next data*/
     rxLength = (int16_t) (rxLength - size);
     rxData = rxData+size;
     /* If there is only one CMD in the buffer, we do not store the result */
     if (number_of_item == 1 && rxLength == 0)
     {
       retVal = accessResult;
     }
     else
     {/* Store the result for each access to be able to report failling access */
       if (txSyncFreeSpace >=0 )
       {
         *txData = accessResult;
         txData = txData+1;
         pHandle->txLength++;
         txSyncFreeSpace--;
         retVal = (accessResult != MCP_CMD_OK) ? MCP_CMD_NOK : retVal;
         if ((accessResult == MCP_ERROR_BAD_DATA_TYPE) || (accessResult == MCP_ERROR_BAD_RAW_FORMAT))
         { /* From this point we are not able to continue to decode CMD buffer*/
           /* We stop the parsing */
           rxLength = 0;
         }
       }
       else
       {
         /* Stop parsing the cmd buffer as no space to answer */
         /* If we reach this state, chances are high the command was badly formated or received */
         rxLength = 0;
         retVal = MCP_ERROR_NO_TXSYNC_SPACE;
       }
     }
  }
  /* If all accesses are fine, just one global MCP_CMD_OK is required*/
  if (retVal == MCP_CMD_OK)
  {
    pHandle->txLength = 0;
  }
  return retVal;
}

__weak uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer+MCP_HEADER_SIZE;
  uint8_t * txData = pHandle->txBuffer;
  uint16_t size = 0;
  uint16_t rxLength = pHandle->rxLength-MCP_HEADER_SIZE;
  uint16_t txSyncFreeSpace = pHandle->pTransportLayer->txSyncMaxPayload-1; /* 1 Byte must be available for MCP CMD status*/
  uint8_t retVal = MCP_CMD_NOK;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE;
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next MCP_ID
     retVal = RI_GetReg (*dataElementID,txData, &size, txSyncFreeSpace);
     if (retVal == MCP_CMD_OK )
     {
       txData = txData+size;
       pHandle->txLength += size;
       txSyncFreeSpace = txSyncFreeSpace-size;
     }
     else
     {
       rxLength = 0;
     }
  }
  return retVal;
}

uint8_t RI_SetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t freeSpace)
{
  uint8_t typeID;
  uint8_t motorID;
  uint8_t retVal = MCP_CMD_OK;
  uint16_t regID = dataID & REG_MASK;
  typeID = dataID & TYPE_MASK;
  motorID = (dataID & MOTOR_MASK)-1;

  MC_Handle_t * pMCI = MC_Core_GetMotorControlHandle( motorID );

  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
    // [SILVIO] uint8_t regdata8 = *data;

    switch (regID)
    {
    case MC_REG_STATUS:
      retVal = MCP_ERROR_RO_REG;
      break;
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 1;
    break;
    }
  case TYPE_DATA_16BIT:
    {
    uint16_t regdata16 = *(uint16_t *)data;
    switch (regID)
    {
    case MC_REG_SPEED_KP:
      pMCI->pid_parameters.kp = regdata16;
      break;
    case MC_REG_SPEED_KI:
      pMCI->pid_parameters.ki = regdata16;
      break;
    case MC_REG_SPEED_KD:
      pMCI->pid_parameters.kd = regdata16;
     break;
    case MC_REG_I_A:
    case MC_REG_I_B:
      retVal = MCP_ERROR_RO_REG;
      break;
        case MC_REG_ENCODER_EL_ANGLE:
    case MC_REG_ENCODER_SPEED:
    case MC_REG_HALL_EL_ANGLE:
    case MC_REG_HALL_SPEED:
      retVal = MCP_ERROR_RO_REG;
     break;
    case MC_REG_DAC_USER1:
     break;
    case MC_REG_DAC_USER2:
     break;
        case MC_REG_STARTUP_CURRENT_REF:
          pMCI->startup_reference = regdata16;
          break;
        case MC_REG_PULSE_VALUE:
          pMCI->pulse_value = regdata16;
          break;

    default:
      retVal = MCP_ERROR_UNKNOWN_REG;

    }
    *size = 2;
    }

    break;

  case TYPE_DATA_32BIT:
    {
    int32_t regdata32 = ((int32_t)(*(int16_t *)&data[2]))<<16 | *(uint16_t *)data;
    switch (regID)
    {
    case MC_REG_FAULTS_FLAGS:
    case MC_REG_SPEED_MEAS:
      retVal = MCP_ERROR_RO_REG;
      break;
    case MC_REG_SPEED_REF:
      pMCI->speed_target_value = regdata32;
      break;
    case MC_REG_STOPLL_EST_BEMF:
    case MC_REG_STOPLL_OBS_BEMF:
    case MC_REG_STOCORDIC_EST_BEMF:
    case MC_REG_STOCORDIC_OBS_BEMF:
      retVal = MCP_ERROR_RO_REG;
      break;
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 4;
    break;
    }
  case TYPE_DATA_STRING:
    {
      const char *charData = (const char *) data;
      char *dummy = (char *) data ;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist*/
      /* It allows to jump to the next command in the buffer */
      RI_MovString (charData, dummy, size, freeSpace);
    }
    break;
  case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *) data;
      *size = rawSize+2; /* The size consumed by the structure is the structure size + 2 bytes used to store the size*/
      uint8_t * rawData = data+2; /* rawData points to the first data (after size extraction) */
      if (*size > freeSpace )
      { /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer construction*/
        *size =0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* this error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
        case MC_REG_GLOBAL_CONFIG:
        case MC_REG_MOTOR_CONFIG:
        case MC_REG_FOCFW_CONFIG:
          retVal = MCP_ERROR_RO_REG;
          break;
        case MC_REG_SPEED_RAMP:
          {
            int32_t rpm;
            rpm = ((int32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData; /* 32 bits access are split into 2x16 bits access */
            if (rpm > 0)
            {
              if (pMCI->direction == 1) MC_Core_Stop(pMCI);
              pMCI->speed_target_command = rpm;
              pMCI->direction = 0;
            }
            else
            {
              if (pMCI->direction == 0) MC_Core_Stop(pMCI);
              pMCI->speed_target_command = - rpm;
              pMCI->direction = 1;
            }
          }
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
      }
    }
    break;
  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    *size =0; /* From this point we are not able anymore to decode the RX buffer*/
  }
  return retVal;
}

uint8_t RI_GetReg (uint16_t dataID, uint8_t * data, uint16_t *size, uint16_t freeSpace )
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t motorID = (dataID & MOTOR_MASK)-1;
  uint16_t regID = dataID & REG_MASK;
  uint8_t retVal = MCP_CMD_OK;

  MC_Handle_t * pMCI = MC_Core_GetMotorControlHandle( motorID );
  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
      if (freeSpace > 0 )
      {
        switch (regID)
        {
        case MC_REG_STATUS:
          *data = MC_Core_GetState(pMCI);
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 1;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_16BIT:
    {
      uint16_t * regdataU16 = (uint16_t *)data;
      int16_t * regdata16 = (int16_t *) data;
      if (freeSpace >= 2 )
      {
        switch (regID)
        {
        case MC_REG_SPEED_KP:
          *regdata16 = pMCI->pid_parameters.kp;
          break;
        case MC_REG_SPEED_KI:
          *regdata16 = pMCI->pid_parameters.ki;
          break;
        case MC_REG_SPEED_KD:
          *regdata16 = pMCI->pid_parameters.kd;
         break;

        case MC_REG_SPEED_KP_DIV:
          *regdataU16 = pMCI->pid_parameters.scaling_shift;
         break;
        case MC_REG_SPEED_KI_DIV:
          *regdataU16 = pMCI->pid_parameters.scaling_shift;
         break;
        case MC_REG_SPEED_KD_DIV:
         break;
        case MC_REG_STARTUP_CURRENT_REF:
          *regdataU16 = pMCI->startup_reference;
          break;
        case MC_REG_PULSE_VALUE:
          *regdataU16 = pMCI->pulse_value;
          break;

        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 2;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_32BIT:
    {
      uint32_t *regdataU32 = (uint32_t *) data;
      int32_t *regdata32 = (int32_t *) data;
      if ( freeSpace >= 4)
      {
        switch (regID)
        {
        case MC_REG_FAULTS_FLAGS:
          *regdataU32 = MC_Core_GetFaultState( pMCI );
          break;
        case MC_REG_SPEED_MEAS:
          if (pMCI->direction == 0) *regdata32 = MC_Core_GetSpeed( pMCI );
          if (pMCI->direction == 1) *regdata32 = - (MC_Core_GetSpeed( pMCI ));
          break;
        case MC_REG_SPEED_REF:
          if (pMCI->direction == 0) *regdata32 = pMCI->speed_target_command;
          if (pMCI->direction == 1) *regdata32 = - (pMCI->speed_target_command);
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 4;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_STRING:
    {
      char *charData = (char *) data;
      switch (regID)
      {
      case MC_REG_FW_VERSION:

        break;
      case MC_REG_CTRL_STAGE_NAME:
        retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
        break;
      case MC_REG_PWR_STAGE_NAME:
        retVal = RI_MovString (PWR_BOARD_NAME[motorID] ,charData, size, freeSpace);
        break;
      case MC_REG_MOTOR_NAME:
        retVal = RI_MovString (MOTOR_NAME[motorID] ,charData, size, freeSpace);
        break;
      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
        *size= 0 ; /* */
      }
    }
    break;
  case TYPE_DATA_RAW:
    {
      uint16_t *rawSize = (uint16_t *) data; /* First 2 bytes of the answer is reserved to the size */
      uint8_t * rawData = data+2;

      switch (regID)
      {
        case MC_REG_GLOBAL_CONFIG:
        *rawSize = sizeof(GlobalConfig_reg_t);
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t) );
        }
        break;
      case MC_REG_MOTOR_CONFIG:
        *rawSize = sizeof(MotorConfig_reg_t);
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, MotorConfig_reg[motorID], sizeof(MotorConfig_reg_t) );
        }
      break;
      case MC_REG_FOCFW_CONFIG:
      break;
      case MC_REG_SPEED_RAMP:
        {
          uint16_t *rpm16p = (uint16_t *) rawData;
          uint16_t *duration = (uint16_t *) &rawData[4];
          int32_t rpm32 = (int32_t)(pMCI->speed_target_command);
          *rpm16p = (uint16_t) rpm32;
          *(rpm16p+1) = (uint16_t) (rpm32>>16);
          *duration = 1; /* Not zero... */
          *rawSize = 6;
        }
        break;
      break;
      case MC_REG_ASYNC_UARTA:
      case MC_REG_ASYNC_UARTB:
      case MC_REG_ASYNC_STLNK:

      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
      }

      /* Size of the answer is size of the data + 2 bytes containing data size*/
      *size = (*rawSize)+2;
    }
    break;
  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    break;
  }
  return retVal;
}

uint8_t RI_MovString (const char * srcString, char * destString, uint16_t *size, uint16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  *size= 1 ; /* /0 is the min String size */
  while ((*srcString != 0) && (*size < maxSize) )
  {
    *destString = *srcString ;
    srcString = srcString+1;
    destString = destString+1;
    *size=*size+1;
  }
  if (*srcString != 0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *destString = 0;
  }
return retVal;
}

uint8_t RI_GetIDSize (uint16_t dataID)
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t result;
  switch (typeID)
  {
    case TYPE_DATA_8BIT:
      result = 1;
      break;
    case TYPE_DATA_16BIT:
      result = 2;
      break;
    case TYPE_DATA_32BIT:
      result = 4;
      break;
    default:
      result=0;
      break;
  }
  return result;
}
__weak uint8_t RI_GetPtrReg (uint16_t dataID, void ** dataPtr)
{
  (void)dataID;
  (void)dataPtr;
  return MCP_CMD_NOK;
}

/************************ (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
