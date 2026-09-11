#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "uart_service.h"
#include "button.h"
#include "ble_nus.h"
#include "comms_protocol.h"
#include "ble_cmd_dispatcher.h"

#define AGV_UART_PORT        UART_NUM_2
#define AGV_PIN_UART_TX      GPIO_NUM_17
#define AGV_PIN_UART_RX      GPIO_NUM_16
#define AGV_UART_BAUDRATE    115200

#define AGV_PIN_BOOT_BTN     GPIO_NUM_0

static const char *TAG = "MAIN";

static void on_boot_button_pressed(void *user_data) {
    (void)user_data;
    ESP_LOGI(TAG, "BOOT button pressed -> Sending 'AGV_MSG_PING' over UART2...");
    uart_service_send_msg(AGV_MSG_PING, NULL, 0);
    if (ble_nus_is_connected()) {
        ble_nus_send_str("[BOOT BTN] PING sent to STM32\r\n");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting AGV ESP32 Binary Gateway firmware...");

    // Initialize NVS Flash (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BLE Command Dispatcher
    ble_cmd_dispatcher_init();

    // Configure and initialize UART service
    const uart_service_config_t uart_cfg = {
        .uart_port       = AGV_UART_PORT,
        .tx_pin          = AGV_PIN_UART_TX,
        .rx_pin          = AGV_PIN_UART_RX,
        .baud_rate       = AGV_UART_BAUDRATE,
        .rx_buffer_size  = 2048,
        .tx_buffer_size  = 1024,
        .task_stack_size = 4096,
        .task_priority   = 10,
        .task_core_id    = 1
    };
    ESP_ERROR_CHECK(uart_service_init(&uart_cfg));

    // Register binary frame callback from UART2 to BLE dispatcher
    uart_service_register_frame_cb(ble_cmd_dispatcher_handle_frame, NULL);

    // Configure and initialize BLE Nordic UART Service (NUS)
    const ble_nus_config_t ble_cfg = {
        .device_name = "AGV_ESP32",
        .rx_callback = ble_cmd_dispatcher_handle_text,
        .user_data   = NULL,
    };
    ESP_ERROR_CHECK(ble_nus_init(&ble_cfg));

    // Configure and initialize BOOT button (GPIO 0, Active LOW)
    const button_config_t btn_cfg = {
        .gpio_num      = AGV_PIN_BOOT_BTN,
        .on_press_down = on_boot_button_pressed,
    };
    ESP_ERROR_CHECK(button_init(&btn_cfg));

    ESP_LOGI(TAG, "AGV System initialized with Binary UART2 & BLE NUS. Ready.");
}
