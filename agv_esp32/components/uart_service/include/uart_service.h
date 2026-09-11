#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cmd_handler_t)(const char *arg, char *response, size_t max_resp_len);

typedef struct {
    const char *command;
    cmd_handler_t handler;
    const char *description;
} cmd_entry_t;

typedef struct {
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
    size_t event_queue_size;
    char pattern_char;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
    BaseType_t task_core_id;
} uart_service_config_t;

#define UART_SERVICE_CONFIG_DEFAULT() { \
    .uart_port = UART_NUM_1,            \
    .tx_pin = GPIO_NUM_17,              \
    .rx_pin = GPIO_NUM_16,              \
    .baud_rate = 115200,                \
    .rx_buffer_size = 1024,             \
    .tx_buffer_size = 1024,             \
    .event_queue_size = 20,             \
    .pattern_char = '\n',               \
    .task_stack_size = 4096,            \
    .task_priority = 10,                \
    .task_core_id = 1                   \
}

/**
 * @brief Initialize UART Service with the specified configuration.
 *        If config is NULL, default configuration is used.
 * 
 * @param config Pointer to configuration struct, or NULL for defaults.
 * @return esp_err_t ESP_OK on success, or error code.
 */
esp_err_t uart_service_init(const uart_service_config_t *config);

/**
 * @brief De-initialize UART Service, stop task and release resources.
 * 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t uart_service_deinit(void);

/**
 * @brief Send raw binary data over UART.
 * 
 * @param data Pointer to data buffer.
 * @param len Length in bytes.
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send(const char *data, size_t len);

/**
 * @brief Send a null-terminated string over UART.
 * 
 * @param str Null-terminated string.
 * @return int Number of bytes sent, or -1 on error.
 */
int uart_service_send_str(const char *str);

/**
 * @brief Register a command table with the command dispatcher.
 * 
 * @param table Array of cmd_entry_t entries.
 * @param table_size Number of entries in table.
 */
void cmd_dispatcher_register(const cmd_entry_t *table, size_t table_size);

#ifdef __cplusplus
}
#endif
