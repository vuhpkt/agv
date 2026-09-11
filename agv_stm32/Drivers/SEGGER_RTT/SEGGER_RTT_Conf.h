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

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#include "main.h"

/*********************************************************************
*
*       Defines, configurable (Tối ưu hóa dung lượng RAM cho AGV)
*
**********************************************************************
*/

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (1)     // Chỉ dùng 1 kênh Terminal duy nhất để tiết kiệm RAM
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (1)     // 1 kênh nhận từ Host (Down Buffer)

#define BUFFER_SIZE_UP                            (4096)  // 4096 bytes cho kênh gửi lên Host (tránh tràn đệm khi gửi dữ liệu 100Hz)
#define BUFFER_SIZE_DOWN                          (128)   // 128 bytes cho kênh nhận lệnh từ Host

#define SEGGER_RTT_PRINTF_BUFFER_SIZE             (64u)   // Bộ đệm format tạm cho hàm printf

#define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP // Không bao giờ làm treo CPU nếu host không kịp đọc

/*********************************************************************
*
*       RTT lock configuration for ARM Cortex-M
*
**********************************************************************
*/

#define SEGGER_RTT_LOCK()               \
  uint32_t _rtt_primask = __get_PRIMASK(); \
  __disable_irq()

#define SEGGER_RTT_UNLOCK()             \
  __set_PRIMASK(_rtt_primask)

#endif
