#include "uart_service.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_SERVICE";

static uart_service_config_t s_config;
static bool s_is_initialized = false;
static volatile bool s_task_running = false;
static TaskHandle_t s_uart_task_handle = NULL;

static agv_frame_rx_cb_t s_frame_cb = NULL;
static void *s_frame_cb_user_data = NULL;

/* Statically allocated protocol parser and frame structures (Zero-Malloc) */
static agv_parser_t s_parser;
static agv_frame_t s_rx_frame;
static uint8_t s_raw_rx_buf[256];

void uart_service_register_frame_cb(agv_frame_rx_cb_t cb, void *user_data) {
    s_frame_cb = cb;
    s_frame_cb_user_data = user_data;
}

static void uart_rx_task(void *pvParameters) {
    (void)pvParameters;
    agv_parser_init(&s_parser);

    ESP_LOGI(TAG, "Binary UART RX task started (Zero-Malloc parser)");

    while (s_task_running) {
        int bytes_read = uart_read_bytes(
            s_config.uart_port,
            s_raw_rx_buf,
            sizeof(s_raw_rx_buf),
            pdMS_TO_TICKS(20)
        );

        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                agv_parse_status_t status = agv_parser_feed_byte(&s_parser, s_raw_rx_buf[i], &s_rx_frame);
                if (status == AGV_PARSE_OK) {
                    if (s_frame_cb != NULL) {
                        s_frame_cb(&s_rx_frame, s_frame_cb_user_data);
                    }
                } else if (status == AGV_PARSE_ERR_CRC) {
                    ESP_LOGW(TAG, "CRC error on incoming frame (MsgID: 0x%02X)", s_parser.msg_id);
                } else if (status == AGV_PARSE_ERR_LENGTH) {
                    ESP_LOGW(TAG, "Length error on incoming frame (> %d)", AGV_PROTOCOL_MAX_PAYLOAD);
                }
            }
        }
    }

    s_uart_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t uart_service_init(const uart_service_config_t *config) {
    if (s_is_initialized) {
        ESP_LOGW(TAG, "UART service already initialized");
        return ESP_OK;
    }

    if (config != NULL) {
        s_config = *config;
    } else {
        const uart_service_config_t default_cfg = UART_SERVICE_CONFIG_DEFAULT();
        s_config = default_cfg;
    }

    if (s_config.rx_buffer_size == 0) s_config.rx_buffer_size = 2048;
    if (s_config.tx_buffer_size == 0) s_config.tx_buffer_size = 1024;
    if (s_config.task_stack_size == 0) s_config.task_stack_size = 4096;
    if (s_config.task_priority == 0) s_config.task_priority = 10;

    const uart_config_t uart_hw_cfg = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    esp_err_t ret = uart_param_config(s_config.uart_port, &uart_hw_cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(s_config.uart_port, s_config.tx_pin, s_config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;
    gpio_set_pull_mode(s_config.rx_pin, GPIO_PULLUP_ONLY);

    ret = uart_driver_install(s_config.uart_port, s_config.rx_buffer_size, s_config.tx_buffer_size, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    s_task_running = true;
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        uart_rx_task,
        "uart_rx_task",
        s_config.task_stack_size,
        NULL,
        s_config.task_priority,
        &s_uart_task_handle,
        s_config.task_core_id
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_rx_task");
        s_task_running = false;
        uart_driver_delete(s_config.uart_port);
        return ESP_FAIL;
    }

    s_is_initialized = true;
    ESP_LOGI(TAG, "Binary UART Service initialized on Port %d (TX: %d, RX: %d, Baud: %d)",
             s_config.uart_port, s_config.tx_pin, s_config.rx_pin, s_config.baud_rate);
    return ESP_OK;
}

esp_err_t uart_service_deinit(void) {
    if (!s_is_initialized) {
        return ESP_OK;
    }

    s_task_running = false;

    int wait_cycles = 50;
    while (s_uart_task_handle != NULL && wait_cycles-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_uart_task_handle != NULL) {
        vTaskDelete(s_uart_task_handle);
        s_uart_task_handle = NULL;
    }

    uart_driver_delete(s_config.uart_port);
    s_is_initialized = false;

    ESP_LOGI(TAG, "UART Service de-initialized");
    return ESP_OK;
}

int uart_service_send_frame(const uint8_t *frame_buf, size_t len) {
    if (!s_is_initialized || frame_buf == NULL || len == 0) {
        return 0;
    }
    return uart_write_bytes(s_config.uart_port, (const char *)frame_buf, len);
}

int uart_service_send_msg(uint8_t msg_id, const void *payload, uint8_t payload_len) {
    if (!s_is_initialized) {
        return -1;
    }

    uint8_t tx_buf[AGV_PROTOCOL_MAX_FRAME_SIZE];
    size_t packed_len = agv_pack_frame(msg_id, payload, payload_len, tx_buf, sizeof(tx_buf));
    if (packed_len == 0) {
        ESP_LOGE(TAG, "Failed to pack frame (msg_id: 0x%02X, len: %u)", msg_id, payload_len);
        return -1;
    }

    return uart_write_bytes(s_config.uart_port, (const char *)tx_buf, packed_len);
}

int uart_service_send_raw(const uint8_t *data, size_t len) {
    if (!s_is_initialized || data == NULL || len == 0) {
        return 0;
    }
    return uart_write_bytes(s_config.uart_port, (const char *)data, len);
}

int uart_service_send_str(const char *str) {
    if (!s_is_initialized || str == NULL) {
        return 0;
    }
    return uart_write_bytes(s_config.uart_port, str, strlen(str));
}
