
#include "mc_stm_types.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "stlink_mcptl.h"

static uint8_t STLNK_TXframeProcess (STLNK_Handle_t *pHandle, uint8_t dataType, void *txBuffer, uint16_t bufferLength);
static void STLNK_sendError (STLNK_Handle_t *pHandle, uint8_t error);

/*TODO: Disable High frequency task is enough */
static inline void disable_hftask_irq()
{
	__disable_irq();
}

/*TODO: Enable High frequency task is enough */
static inline void enable_hftask_irq()
{
	__enable_irq();
}

void STLNK_init (STLNK_Handle_t *pHandle)
{
  stlnkCtrl.maxSyncPayloadSize = MCP_TX_SYNC_PAYLOAD_MAX;
  pHandle->_Super.txSyncMaxPayload = MCP_TX_SYNC_PAYLOAD_MAX;
 }

bool STLNK_getBuffer (MCTL_Handle_t *pSupHandle, void **buffer,  uint8_t syncAsync)
{
  STLNK_Handle_t *pHandle = (STLNK_Handle_t *) pSupHandle;
  bool result = true;
  if (syncAsync == MCTL_SYNC)
  {
    if (pHandle->syncBuffer.state <= writeLock ) /* Possible values are free or writeLock*/
    {
      *buffer = pHandle->syncBuffer.buffer;
      pHandle->syncBuffer.state = writeLock;
    }
    else
    {
      result = false;
    }
  }

  else /* Asynchronous buffer request */
  {
    result = false; /* Asynchronous buffer not supported */
  }
return result;
}

uint8_t STLNK_sendPacket (MCTL_Handle_t *pSupHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync)
{
  uint8_t *trailerPacket;
  uint16_t totalLength;
  uint8_t result = STLNK_OK;
  STLNK_Handle_t *pHandle = (STLNK_Handle_t *) pSupHandle;

  trailerPacket = (uint8_t *) txBuffer;
  trailerPacket = trailerPacket + txDataLength; /* */

    if (syncAsync == MCTL_SYNC )
    {
      if ( pSupHandle->MCP_PacketAvailable)
      {
        /* We need to send an acknowledge packet with answer */
        pSupHandle-> MCP_PacketAvailable = false; /* CMD from master is processed*/

        /*Add CRC (to be implemented) */
        *(trailerPacket) = (uint8_t) 0xCA; /* Dummy CRC */
        *(trailerPacket+1) = (uint8_t) 0xFE; /* Dummy CRC */
        *(trailerPacket+2) = (uint8_t) STLNK_RESPONSE_PACKET;
        totalLength = txDataLength + STLNK_CRC_SIZE + 1; /* 1 additional byte for packet type */
      }
      else
      {
        result = STLNK_SYNC_NOT_EXPECTED;
      }
    }
    else /* Packet is an ASync packet*/
    {       result = STLNK_ASYNC_NOT_EXPECTED;
    }
    if (result == STLNK_OK)
    {
      result = STLNK_TXframeProcess (pHandle, syncAsync, txBuffer, totalLength );
    }
    return result;
}

void STLNK_sendError (STLNK_Handle_t *pHandle, uint8_t error)
{
  uint32_t * packet = (uint32_t*) pHandle->syncBuffer.buffer;
  *packet = STLNK_ERROR_PACKET << 28 | error ;
  STLNK_TXframeProcess (pHandle, MCTL_SYNC, &pHandle->syncBuffer.buffer, STLNK_ERROR_PACKET_SIZE);
}

uint8_t STLNK_TXframeProcess (STLNK_Handle_t *pHandle, uint8_t syncAsync, void *txBuffer, uint16_t bufferLength)
{
  (void)txBuffer;
  (void)bufferLength;
  uint8_t result = STLNK_OK;
  disable_hftask_irq();
  if (pHandle->lockBuffer == NULL ) /* Communication Ip free to send data*/
  {
    if (syncAsync == MCTL_SYNC )
    {
      pHandle->syncBuffer.state = readLock;
      pHandle->lockBuffer = (void *) &pHandle->syncBuffer;
      stlnkCtrl.bufferID =  0; /* Useless here. Only one Sync buffer exists*/
      stlnkCtrl.bufferSize = bufferLength;
      stlnkCtrl.bufferType = STLNK_SYNC_BUFFER;
    }
    else
    {
    /* No other packets type supported */
    }
    /* Enable HF task It */
    enable_hftask_irq();
    /* */
  }
  else /* HW resource busy, saving packet to sent it once resource will be freed*/
  {
	  enable_hftask_irq();

    if (syncAsync == MCTL_SYNC )
    {
      if ( pHandle -> syncBuffer.state != writeLock ) {
        result = STLNK_BUFFER_ERROR;
      }
      else {
        pHandle -> syncBuffer.state = pending;
        pHandle -> syncBuffer.length = bufferLength;
      }
    }
    else
    { /* No other packet types supported*/
    }
  }
return result;
}

void STLNK_HWDataTransmittedIT (STLNK_Handle_t *pHandle )
{
  /* first, free the buffer just sent */
    MCTL_Buff_t * tempBuff = (MCTL_Buff_t *) pHandle -> lockBuffer;
    tempBuff->state = free;

  /* Second prepare transfer of pending buffer */
  if ( pHandle -> syncBuffer.state == pending )
  {
    pHandle->lockBuffer = (void *) &pHandle->syncBuffer;
    /* */
    pHandle ->syncBuffer.state = readLock;
    stlnkCtrl.bufferSize = pHandle->syncBuffer.length;
    stlnkCtrl.bufferType = STLNK_SYNC_BUFFER;
    /* */
  }
  else
  {
		  pHandle->lockBuffer = NULL;
		  stlnkCtrl.bufferType = STLNK_NO_BUFFER;
  }
}

uint8_t* STLNK_RXframeProcess (MCTL_Handle_t *pSupHandle, uint16_t *packetLength)
{
  uint8_t* result = NULL;
  STLNK_Handle_t * pHandle = (STLNK_Handle_t *) pSupHandle;
  bool validCRCData = true;
  bool validHeader = true;

  /* Length of the packet send by the STLink is stored in the two first bytes*/
  *packetLength = * ((uint16_t *) pHandle->rxBuffer);

  if (*packetLength != 0)
  {
    *((uint16_t *) pHandle->rxBuffer) = 0;  /* Consumes new packet by clearing the packet size*/
    if (validHeader)
    {
      if (validCRCData)
      {
        pSupHandle ->MCP_PacketAvailable = true; /* Will be consumed in FCP_sendPacket */
        result = pHandle->rxBuffer+4; /* Header is 4 bytes 2x16bits size*/
      }
      else
      {
        STLNK_sendError (pHandle, STLNK_BAD_CRC_DATA);
      }
    }
    else
    {
       STLNK_sendError (pHandle, STLNK_BAD_PACKET_SIZE);
    }
  }

  return result;
}
