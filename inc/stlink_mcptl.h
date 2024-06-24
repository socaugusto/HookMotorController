
#ifndef stlnk_protocol_h
#define stlnk_protocol_h

#include "stdint.h"
#include "mcptl.h"

#define STLNK_ERROR_PACKET_SIZE 4
#define STLNK_CRC_SIZE 2

#define STLNK_OK 0
#define STLNK_SYNC_NOT_EXPECTED 1
#define STLNK_BUFFER_ERROR 2
#define STLNK_ASYNC_NOT_EXPECTED 3

#define STLNK_SYNC_PACKET_TYPE_SIZE 1 /*Sync Packet Types are STLNK_ERROR_PACKET or STLNK_RESPONSE_PACKET */
#define STLNK_ERROR_PACKET 0xF
#define STLNK_RESPONSE_PACKET 0xA
/* STLNK Protocol errors */
#define STLNK_BAD_CRC_DATA    1
#define STLNK_BAD_PACKET_SIZE 2

#define STLNK_ASYNC_BUFFER 0xA /* To be replaced by MCTL_ASYNC once updated in MC Pilot */
#define STLNK_SYNC_BUFFER  0x6 /* To be replaced by MCTL_SYNC  once updated in MC Pilot */
#define STLNK_NO_BUFFER  0x0

typedef struct {
        uint8_t bufferType; /* Allow Stlink layer to descrimine sync or Async buffers*/
        uint8_t bufferID; /* used to select asyncBuffer A or B */
        uint16_t bufferSize;
        uint8_t * rxBuffer;
        uint8_t * syncBuffer;
        uint8_t * asyncBufferA;
        uint8_t * asyncBufferB;
        uint16_t maxSyncPayloadSize;
        uint16_t maxAsyncPayloadSize;
} STLNK_Control_t;

typedef struct
{
  MCTL_Handle_t _Super; /* 17 bytes */
  void * lockBuffer;
  uint8_t * rxBuffer;
  MCTL_Buff_t syncBuffer;
} STLNK_Handle_t;

bool STLNK_getBuffer (MCTL_Handle_t *pHandle, void **buffer,  uint8_t syncAsync);
uint8_t STLNK_sendPacket (MCTL_Handle_t *pHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync);
uint8_t* STLNK_RXframeProcess (MCTL_Handle_t *pHandle, uint16_t *packetLength);
void STLNK_HWDataTransmittedIT (STLNK_Handle_t *pHandle );
void STLNK_init (STLNK_Handle_t *pHandle);

#endif
