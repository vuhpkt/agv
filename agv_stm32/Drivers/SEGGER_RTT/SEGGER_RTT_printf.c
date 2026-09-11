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
#include <stdarg.h>

#define FORMAT_FLAG_LEFT_JUSTIFY   (1u << 0)
#define FORMAT_FLAG_PAD_ZERO       (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN     (1u << 2)
#define FORMAT_FLAG_ALTERNATE      (1u << 3)

typedef struct {
  char*    pBuffer;
  unsigned BufferSize;
  unsigned Cnt;
  unsigned RTTBufferIndex;
} BUFFER_DESC;

static void _StoreChar(BUFFER_DESC * pDesc, char c) {
  unsigned Cnt = pDesc->Cnt;
  if (Cnt < pDesc->BufferSize) {
    *(pDesc->pBuffer + Cnt) = c;
    pDesc->Cnt = Cnt + 1;
  }
  if (pDesc->Cnt >= pDesc->BufferSize) {
    SEGGER_RTT_Write(pDesc->RTTBufferIndex, pDesc->pBuffer, pDesc->Cnt);
    pDesc->Cnt = 0;
  }
}

static void _PrintUnsigned(BUFFER_DESC * pDesc, unsigned v, unsigned Base, unsigned FieldWidth, unsigned FormatFlags) {
  static const char _aHexDigits[] = "0123456789ABCDEF";
  char aBuffer[16];
  unsigned NumDigits = 0;

  do {
    aBuffer[NumDigits++] = _aHexDigits[v % Base];
    v /= Base;
  } while (v > 0);

  if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0) {
    while (NumDigits < FieldWidth) {
      _StoreChar(pDesc, (FormatFlags & FORMAT_FLAG_PAD_ZERO) ? '0' : ' ');
      FieldWidth--;
    }
  }

  while (NumDigits > 0) {
    _StoreChar(pDesc, aBuffer[--NumDigits]);
  }

  while (FieldWidth > 0) {
    _StoreChar(pDesc, ' ');
    FieldWidth--;
  }
}

static void _PrintInt(BUFFER_DESC * pDesc, int v, unsigned Base, unsigned FieldWidth, unsigned FormatFlags) {
  if (v < 0) {
    _StoreChar(pDesc, '-');
    v = -v;
  } else if (FormatFlags & FORMAT_FLAG_PRINT_SIGN) {
    _StoreChar(pDesc, '+');
  }
  _PrintUnsigned(pDesc, (unsigned)v, Base, FieldWidth, FormatFlags);
}

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList) {
  char acBuffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  BUFFER_DESC BufferDesc;
  char c;

  BufferDesc.pBuffer        = acBuffer;
  BufferDesc.BufferSize     = SEGGER_RTT_PRINTF_BUFFER_SIZE;
  BufferDesc.Cnt            = 0;
  BufferDesc.RTTBufferIndex = BufferIndex;

  do {
    c = *sFormat++;
    if (c == 0) {
      break;
    }
    if (c == '%') {
      unsigned FieldWidth  = 0;
      unsigned FormatFlags = 0;

      c = *sFormat++;
      if (c == '0') {
        FormatFlags |= FORMAT_FLAG_PAD_ZERO;
        c = *sFormat++;
      }
      while ((c >= '0') && (c <= '9')) {
        FieldWidth = FieldWidth * 10 + (c - '0');
        c = *sFormat++;
      }
      if (c == 'l') { // Skip 'l' modifier for %lu / %ld
        c = *sFormat++;
      }

      switch (c) {
      case 'c': {
        char val = (char)va_arg(*pParamList, int);
        _StoreChar(&BufferDesc, val);
        break;
      }
      case 'd':
      case 'i': {
        int val = va_arg(*pParamList, int);
        _PrintInt(&BufferDesc, val, 10, FieldWidth, FormatFlags);
        break;
      }
      case 'u': {
        unsigned val = va_arg(*pParamList, unsigned);
        _PrintUnsigned(&BufferDesc, val, 10, FieldWidth, FormatFlags);
        break;
      }
      case 'x':
      case 'X': {
        unsigned val = va_arg(*pParamList, unsigned);
        _PrintUnsigned(&BufferDesc, val, 16, FieldWidth, FormatFlags);
        break;
      }
      case 's': {
        const char * s = va_arg(*pParamList, const char *);
        while (*s) {
          _StoreChar(&BufferDesc, *s++);
        }
        break;
      }
      case '%': {
        _StoreChar(&BufferDesc, '%');
        break;
      }
      default:
        break;
      }
    } else {
      _StoreChar(&BufferDesc, c);
    }
  } while (1);

  if (BufferDesc.Cnt > 0) {
    SEGGER_RTT_Write(BufferIndex, acBuffer, BufferDesc.Cnt);
  }
  return 0;
}

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...) {
  int r;
  va_list ParamList;

  va_start(ParamList, sFormat);
  r = SEGGER_RTT_vprintf(BufferIndex, sFormat, &ParamList);
  va_end(ParamList);
  return r;
}

