/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2019 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer for embedded targets         *
*                                                                    *
**********************************************************************
*/

#include "SEGGER_RTT.h"
#include <string.h>

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static char _acUpBuffer  [BUFFER_SIZE_UP];
static char _acDownBuffer[BUFFER_SIZE_DOWN];

SEGGER_RTT_CB _SEGGER_RTT = {
  "SEGGER RTT",
  SEGGER_RTT_MAX_NUM_UP_BUFFERS,
  SEGGER_RTT_MAX_NUM_DOWN_BUFFERS,
  {{ "Terminal", &_acUpBuffer[0],   sizeof(_acUpBuffer),   0, 0, SEGGER_RTT_MODE_DEFAULT }},
  {{ "Terminal", &_acDownBuffer[0], sizeof(_acDownBuffer), 0, 0, SEGGER_RTT_MODE_DEFAULT }}
};

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/

static unsigned _WriteNoCheck(SEGGER_RTT_BUFFER_UP* pRing, const char* pData, unsigned NumBytes) {
  unsigned NumBytesToWrite;
  unsigned WrOff;
  unsigned RdOff;
  unsigned RemBytes = NumBytes;

  WrOff = pRing->WrOff;
  RdOff = pRing->RdOff;

  // 1. Write from WrOff to end of buffer
  if (RdOff <= WrOff) {
    NumBytesToWrite = pRing->SizeOfBuffer - WrOff;
    if (RdOff == 0) {
      NumBytesToWrite--; // Don't let WrOff hit RdOff if RdOff is at index 0
    }
    if (NumBytesToWrite > RemBytes) {
      NumBytesToWrite = RemBytes;
    }
    if (NumBytesToWrite > 0) {
      memcpy(pRing->pBuffer + WrOff, pData, NumBytesToWrite);
      pData    += NumBytesToWrite;
      RemBytes -= NumBytesToWrite;
      WrOff    += NumBytesToWrite;
      if (WrOff >= pRing->SizeOfBuffer) {
        WrOff = 0;
      }
    }
  }

  // 2. If we still have data and WrOff wrapped to 0 while RdOff > 0, write from 0 to RdOff - 1
  if (RemBytes > 0 && WrOff < RdOff) {
    NumBytesToWrite = RdOff - WrOff - 1;
    if (NumBytesToWrite > RemBytes) {
      NumBytesToWrite = RemBytes;
    }
    if (NumBytesToWrite > 0) {
      memcpy(pRing->pBuffer + WrOff, pData, NumBytesToWrite);
      pData    += NumBytesToWrite;
      RemBytes -= NumBytesToWrite;
      WrOff    += NumBytesToWrite;
    }
  }

  pRing->WrOff = WrOff;
  return NumBytes - RemBytes;
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

void SEGGER_RTT_Init(void) {
  // Initialization done via static structure init
}

unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  SEGGER_RTT_BUFFER_UP* pRing;
  unsigned NumBytesWritten = 0;
  unsigned FreeSpace;
  unsigned WrOff;
  unsigned RdOff;

  if (BufferIndex >= (unsigned)_SEGGER_RTT.MaxNumUpBuffers || pBuffer == NULL || NumBytes == 0) {
    return 0;
  }
  pRing = &_SEGGER_RTT.aUp[BufferIndex];
  if (pRing->pBuffer == NULL || pRing->SizeOfBuffer == 0) {
    return 0;
  }

  SEGGER_RTT_LOCK();
  WrOff = pRing->WrOff;
  RdOff = pRing->RdOff;

  // Calculate TOTAL free space in circular buffer
  if (RdOff <= WrOff) {
    FreeSpace = pRing->SizeOfBuffer - 1 - (WrOff - RdOff);
  } else {
    FreeSpace = RdOff - WrOff - 1;
  }

  if (pRing->Flags == SEGGER_RTT_MODE_NO_BLOCK_SKIP) {
    if (NumBytes > FreeSpace) {
      SEGGER_RTT_UNLOCK();
      return 0;
    }
  }

  NumBytesWritten = _WriteNoCheck(pRing, (const char*)pBuffer, NumBytes);
  SEGGER_RTT_UNLOCK();

  return NumBytesWritten;
}

unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s) {
  if (s == NULL) return 0;
  return SEGGER_RTT_Write(BufferIndex, s, strlen(s));
}

unsigned SEGGER_RTT_Read(unsigned BufferIndex, void* pData, unsigned BufferSize) {
  SEGGER_RTT_BUFFER_DOWN* pRing;
  unsigned NumBytesRem;
  unsigned NumBytesRead;
  unsigned RdOff;
  unsigned WrOff;
  char*    pDest;

  if (BufferIndex >= (unsigned)_SEGGER_RTT.MaxNumDownBuffers || pData == NULL || BufferSize == 0) {
    return 0;
  }
  pRing = &_SEGGER_RTT.aDown[BufferIndex];
  if (pRing->pBuffer == NULL || pRing->SizeOfBuffer == 0) {
    return 0;
  }

  pDest = (char*)pData;
  NumBytesRead = 0;
  SEGGER_RTT_LOCK();
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;

  if (RdOff <= WrOff) {
    NumBytesRem = WrOff - RdOff;
  } else {
    NumBytesRem = pRing->SizeOfBuffer - RdOff + WrOff;
  }

  if (NumBytesRem > BufferSize) {
    NumBytesRem = BufferSize;
  }

  while (NumBytesRem > 0) {
    *pDest++ = *(pRing->pBuffer + RdOff++);
    if (RdOff >= pRing->SizeOfBuffer) {
      RdOff = 0;
    }
    NumBytesRead++;
    NumBytesRem--;
  }
  pRing->RdOff = RdOff;
  SEGGER_RTT_UNLOCK();

  return NumBytesRead;
}

unsigned SEGGER_RTT_HasKey(void) {
  SEGGER_RTT_BUFFER_DOWN* pRing = &_SEGGER_RTT.aDown[0];
  return (pRing->WrOff != pRing->RdOff);
}

unsigned SEGGER_RTT_GetKey(void) {
  char c;
  if (SEGGER_RTT_Read(0, &c, 1) == 1) {
    return (unsigned)(unsigned char)c;
  }
  return 0;
}
