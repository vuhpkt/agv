/**
 * @file app_comms.h
 * @brief Giao tiếp UART nhị phân AGV với ESP32
 */

#ifndef APP_COMMS_H
#define APP_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void App_Comms_Init(void);
void App_Comms_Process(void);

void App_Comms_SendLog(const char *fmt, ...);
void App_Comms_SendAck(uint8_t acked_msg_id, uint8_t status);
void App_Comms_SendMissionStatus(uint8_t state, uint8_t current_step);
void App_Comms_SendEncData(int32_t left, int32_t right);
void App_Comms_SendTelemetry(int16_t left_rpm_x10, int16_t right_rpm_x10, uint8_t ir_mask);

/** Macro log nhị phân dùng chung toàn hệ thống thay thế SEGGER_RTT_printf */
#define AGV_LOG(...) App_Comms_SendLog(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMS_H */