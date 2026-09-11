#pragma once

#include <stddef.h>
#include <stdint.h>
#include "comms_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLE command dispatcher.
 */
void ble_cmd_dispatcher_init(void);

/**
 * @brief Callback for incoming text lines from BLE NUS RX characteristic.
 *        Parses high-level text commands and forwards corresponding binary frames
 *        to STM32 via uart_service.
 * 
 * @param data Raw byte buffer received from BLE characteristic.
 * @param len Length in bytes.
 * @param user_data User context pointer.
 */
void ble_cmd_dispatcher_handle_text(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Callback for incoming binary frames from STM32 via uart_service.
 *        Formats binary frames into structured text messages with prefixes
 *        ([RESP], [ACK], [STM32 LOG], [MISSION], [ENCODER], DATA:...) and notifies the BLE client.
 * 
 * @param frame Validated binary frame received from STM32.
 * @param user_data User context pointer.
 */
void ble_cmd_dispatcher_handle_frame(const agv_frame_t *frame, void *user_data);

#ifdef __cplusplus
}
#endif
