#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "comms_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback function type for received binary AGV frames.
 */
typedef void (*agv_frame_rx_cb_t)(const agv_frame_t *frame, void *user_data);

/**
 * @brief UART service configuration parameters.
 */
typedef struct {
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
    BaseType_t task_core_id;
} uart_service_config_t;

#define UART_SERVICE_CONFIG_DEFAULT() { \
    .uart_port = UART_NUM_2,            \
    .tx_pin = GPIO_NUM_17,              \
    .rx_pin = GPIO_NUM_16,              \
    .baud_rate = 115200,                \
    .rx_buffer_size = 2048,             \
    .tx_buffer_size = 1024,             \
    .task_stack_size = 4096,            \
    .task_priority = 10,                \
    .task_core_id = 1                   \
}

/**
 * @brief Initialize UART Service with binary frame streaming parser task.
 *        If config is NULL, default configuration (UART2, GPIO16/17, 115200) is used.
 * 
 * @param config Pointer to configuration struct, or NULL for defaults.
 * @return esp_err_t ESP_OK on success, or error code.
 */
esp_err_t uart_service_init(const uart_service_config_t *config);

/**
 * @brief De-initialize UART Service, stop RX task and release resources.
 * 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t uart_service_deinit(void);

/**
 * @brief Register a callback for incoming binary frames validated by CRC16.
 * 
 * @param cb Callback function to invoke when a valid frame arrives.
 * @param user_data User context pointer passed to callback.
 */
void uart_service_register_frame_cb(agv_frame_rx_cb_t cb, void *user_data);

/**
 * @brief Send a pre-packaged binary frame over UART.
 * 
 * @param frame_buf Pointer to packaged frame bytes.
 * @param len Length in bytes.
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send_frame(const uint8_t *frame_buf, size_t len);

/**
 * @brief Package a payload into an AGV protocol frame and send over UART.
 * 
 * @param msg_id Message ID (agv_msg_id_t).
 * @param payload Pointer to payload buffer, or NULL if length is 0.
 * @param payload_len Payload length in bytes (0..128).
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send_msg(uint8_t msg_id, const void *payload, uint8_t payload_len);

/**
 * @brief Send raw bytes over UART.
 * 
 * @param data Pointer to data buffer.
 * @param len Length in bytes.
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send_raw(const uint8_t *data, size_t len);

/**
 * @brief Send a null-terminated string over UART.
 * 
 * @param str Null-terminated string.
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send_str(const char *str);

#ifdef __cplusplus
}
#endif
