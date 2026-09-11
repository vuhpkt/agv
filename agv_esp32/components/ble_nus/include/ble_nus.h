#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, size_t len, void *user_data);

typedef struct {
    const char *device_name;          // Advertised BLE device name (default: "AGV_ESP32")
    ble_nus_rx_cb_t rx_callback;      // Callback invoked when data is received from BLE Central
    void *user_data;                  // Optional context pointer passed to rx_callback
} ble_nus_config_t;

#define BLE_NUS_CONFIG_DEFAULT() {    \
    .device_name = "AGV_ESP32",       \
    .rx_callback = NULL,              \
    .user_data = NULL                 \
}

/**
 * @brief Initialize BLE Nordic UART Service (NUS) with NimBLE stack.
 * 
 * @param config Pointer to ble_nus_config_t structure, or NULL for default configuration.
 * @return esp_err_t ESP_OK on success, or error code.
 */
esp_err_t ble_nus_init(const ble_nus_config_t *config);

/**
 * @brief Send raw binary data over BLE NUS TX characteristic (Notification).
 * 
 * @param data Pointer to data buffer.
 * @param len Length in bytes.
 * @return int Number of bytes sent, or -1 on error.
 */
int ble_nus_send(const uint8_t *data, size_t len);

/**
 * @brief Send a null-terminated string over BLE NUS TX characteristic.
 * 
 * @param str Null-terminated string.
 * @return int Number of bytes sent, or -1 on error.
 */
int ble_nus_send_str(const char *str);

/**
 * @brief Check if a BLE Central client is currently connected and has notifications enabled.
 * 
 * @return true if connected and subscribed to notifications, false otherwise.
 */
bool ble_nus_is_connected(void);

#ifdef __cplusplus
}
#endif

