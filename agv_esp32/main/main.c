#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "uart_service.h"
#include "button.h"
#include "ble_nus.h"

#define AGV_UART_PORT        UART_NUM_2
#define AGV_PIN_UART_TX      GPIO_NUM_17
#define AGV_PIN_UART_RX      GPIO_NUM_16
#define AGV_UART_BAUDRATE    115200

#define AGV_PIN_BOOT_BTN     GPIO_NUM_0

static const char *TAG = "MAIN";

static void handle_ping(const char *arg, char *res, size_t max_len) {
    (void)arg;
    snprintf(res, max_len, "OK: PONG");
}

static void handle_echo(const char *arg, char *res, size_t max_len) {
    if (arg != NULL && arg[0] != '\0') {
        snprintf(res, max_len, "OK: %s", arg);
    } else {
        snprintf(res, max_len, "OK:");
    }
}

static const cmd_entry_t g_system_commands[] = {
    {"PING", &handle_ping, "Ping test connection"},
    {"ECHO", &handle_echo, "Echo back argument"},
};

static void on_boot_button_pressed(void *user_data) {
    (void)user_data;
    ESP_LOGI(TAG, "BOOT button pressed -> Sending 'PING' over UART and BLE...");
    uart_service_send_str("PING\r\n");
    if (ble_nus_is_connected()) {
        ble_nus_send_str("PING\r\n");
    }
}

static void on_ble_data_received(const uint8_t *data, size_t len, void *user_data) {
    (void)user_data;
    char line[256];
    size_t copy_len = (len < sizeof(line) - 1) ? len : (sizeof(line) - 1);
    memcpy(line, data, copy_len);
    line[copy_len] = '\0';

    line[strcspn(line, "\r\n")] = '\0';
    if (line[0] == '\0') return;

    if (strcasecmp(line, "PING") == 0) {
        ble_nus_send_str("OK: PONG\r\n");
        uart_service_send_str("PING\r\n")
    } else if (strncasecmp(line, "ECHO ", 5) == 0) {
        char res[256];
        snprintf(res, sizeof(res), "OK: %s\r\n", line + 5);
        ble_nus_send_str(res);
    } else {
        // Forward unknown commands to UART
        ESP_LOGI(TAG, "Forwarding BLE command to UART: %s", line);
        uart_service_send_str(line);
        uart_service_send_str("\r\n");
        ble_nus_send_str("OK: FORWARDED\r\n");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting AGV ESP32 firmware...");

    // Initialize NVS Flash (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Register command table
    cmd_dispatcher_register(g_system_commands, sizeof(g_system_commands) / sizeof(cmd_entry_t));

    // Configure and initialize UART service
    const uart_service_config_t uart_cfg = {
        .uart_port  = AGV_UART_PORT,
        .tx_pin     = AGV_PIN_UART_TX,
        .rx_pin     = AGV_PIN_UART_RX,
        .baud_rate  = AGV_UART_BAUDRATE,
    };
    ESP_ERROR_CHECK(uart_service_init(&uart_cfg));

    // Configure and initialize BLE Nordic UART Service (NUS)
    const ble_nus_config_t ble_cfg = {
        .device_name = "AGV_ESP32",
        .rx_callback = on_ble_data_received,
        .user_data   = NULL,
    };
    ESP_ERROR_CHECK(ble_nus_init(&ble_cfg));

    // Configure and initialize BOOT button (GPIO 0, Active LOW)
    const button_config_t btn_cfg = {
        .gpio_num      = AGV_PIN_BOOT_BTN,
        .on_press_down = on_boot_button_pressed,
    };
    ESP_ERROR_CHECK(button_init(&btn_cfg));

    ESP_LOGI(TAG, "AGV System initialized with UART & BLE NUS. Ready.");
}
