#include "uart_service.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_SERVICE";

static uart_service_config_t s_config;
static bool s_is_initialized = false;
static volatile bool s_task_running = false;
static TaskHandle_t s_uart_task_handle = NULL;
static QueueHandle_t s_uart_queue = NULL;
static const cmd_entry_t *s_cmd_table = NULL;
static size_t s_cmd_table_size = 0;

void cmd_dispatcher_register(const cmd_entry_t *table, size_t table_size) {
    s_cmd_table = table;
    s_cmd_table_size = table_size;
}

static void cmd_dispatcher_process(char *line) {
    if (line == NULL) return;

    line[strcspn(line, "\r\n")] = '\0';
    if (line[0] == '\0') return;

    ESP_LOGI(TAG, "[UART RX] %s", line);

    char *saveptr = NULL;
    char *cmd_name = strtok_r(line, " ", &saveptr);
    char *args = strtok_r(NULL, "", &saveptr);

    char response[256] = {0};
    bool is_matched = false;

    if (s_cmd_table && cmd_name) {
        for (size_t i = 0; i < s_cmd_table_size; i++) {
            if (s_cmd_table[i].command && strcasecmp(cmd_name, s_cmd_table[i].command) == 0) {
                is_matched = true;
                if (s_cmd_table[i].handler) {
                    s_cmd_table[i].handler(args, response, sizeof(response));
                }
                break;
            }
        }
    }

    if (is_matched) {
        if (response[0] != '\0') {
            uart_service_send_str(response);
            uart_service_send_str("\r\n");
        }
    }
}

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *rx_buffer = (uint8_t *) malloc(s_config.rx_buffer_size);
    if (rx_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        s_uart_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (s_task_running) {
        if (xQueueReceive(s_uart_queue, (void *)&event, pdMS_TO_TICKS(100))) {
            if (!s_task_running) {
                break;
            }
            memset(rx_buffer, 0, s_config.rx_buffer_size);

            switch (event.type) {
                case UART_PATTERN_DET: {
                    int pos = uart_pattern_pop_pos(s_config.uart_port);

                    if (pos == -1) {
                        uart_flush_input(s_config.uart_port);
                    } else {
                        // Guard against buffer overflow: read at most rx_buffer_size - 1
                        size_t bytes_to_read = (pos + 1 < s_config.rx_buffer_size) ? (size_t)(pos + 1) : (s_config.rx_buffer_size - 1);
                        int read_len = uart_read_bytes(s_config.uart_port, rx_buffer, bytes_to_read, pdMS_TO_TICKS(100));
                        if (read_len > 0) {
                            rx_buffer[read_len] = '\0';
                            cmd_dispatcher_process((char *)rx_buffer);
                        }
                        if (pos + 1 >= s_config.rx_buffer_size) {
                            ESP_LOGW(TAG, "Line exceeded buffer size (%u bytes), flushed input", (unsigned)s_config.rx_buffer_size);
                            uart_flush_input(s_config.uart_port);
                        }
                    }
                    break;
                }
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring buffer/FIFO full - flushing RX");
                    uart_flush_input(s_config.uart_port);
                    xQueueReset(s_uart_queue);
                    break;
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART Error: %d", event.type);
                    break;
                default:
                    break;
            }
        }
    }

    free(rx_buffer);
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

    if (s_config.rx_buffer_size == 0) s_config.rx_buffer_size = 1024;
    if (s_config.tx_buffer_size == 0) s_config.tx_buffer_size = 1024;
    if (s_config.event_queue_size == 0) s_config.event_queue_size = 20;
    if (s_config.task_stack_size == 0) s_config.task_stack_size = 4096;
    if (s_config.task_priority == 0) s_config.task_priority = 10;
    if (s_config.pattern_char == '\0') s_config.pattern_char = '\n';

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

    ret = uart_driver_install(s_config.uart_port, s_config.rx_buffer_size * 2, s_config.tx_buffer_size * 2, s_config.event_queue_size, &s_uart_queue, 0);
    if (ret != ESP_OK) return ret;

    ret = uart_enable_pattern_det_baud_intr(s_config.uart_port, s_config.pattern_char, 1, 9, 0, 0);
    if (ret != ESP_OK) {
        uart_driver_delete(s_config.uart_port);
        s_uart_queue = NULL;
        return ret;
    }

    ret = uart_pattern_queue_reset(s_config.uart_port, s_config.event_queue_size);
    if (ret != ESP_OK) {
        uart_driver_delete(s_config.uart_port);
        s_uart_queue = NULL;
        return ret;
    }

    s_task_running = true;
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        uart_event_task,
        "uart_event_task",
        s_config.task_stack_size,
        NULL,
        s_config.task_priority,
        &s_uart_task_handle,
        s_config.task_core_id
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_event_task");
        s_task_running = false;
        uart_driver_delete(s_config.uart_port);
        s_uart_queue = NULL;
        return ESP_FAIL;
    }

    s_is_initialized = true;
    ESP_LOGI(TAG, "UART Service initialized on Port: %d (TX: %d, RX: %d, Baud: %d)",
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
    s_uart_queue = NULL;
    s_is_initialized = false;

    ESP_LOGI(TAG, "UART Service de-initialized");
    return ESP_OK;
}

int uart_service_send(const char *data, size_t len) {
    if (!s_is_initialized || data == NULL || len == 0) {
        return 0;
    }

    // Print clean log by trimming trailing CRLF in log preview
    int print_len = (int)len;
    while (print_len > 0 && (data[print_len - 1] == '\n' || data[print_len - 1] == '\r')) {
        print_len--;
    }
    if (print_len > 0) {
        ESP_LOGI(TAG, "[UART TX] %.*s", print_len, data);
    }

    return uart_write_bytes(s_config.uart_port, data, len);
}

int uart_service_send_str(const char *str) {
    if (!s_is_initialized || str == NULL) {
        return 0;
    }
    return uart_service_send(str, strlen(str));
}
