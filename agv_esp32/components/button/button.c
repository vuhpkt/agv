#include "button.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG "BUTTON"
#define MAX_BUTTONS 4
#define BUTTON_TASK_STACK_SIZE 3072
#define BUTTON_TASK_PRIORITY 10

typedef struct {
    gpio_num_t gpio_num;
    button_active_level_t active_level;
    uint32_t debounce_ms;
    button_callback_t on_press_down;
    void *user_data;
    TickType_t last_press_tick;
    bool is_registered;
} button_entry_t;

static button_entry_t s_buttons[MAX_BUTTONS] = {0};
static QueueHandle_t s_button_queue = NULL;
static TaskHandle_t s_button_task_handle = NULL;
static bool s_isr_service_installed = false;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    gpio_num_t gpio_num = (gpio_num_t)(intptr_t)arg;
    BaseType_t high_task_woken = pdFALSE;
    if (s_button_queue != NULL) {
        xQueueSendFromISR(s_button_queue, &gpio_num, &high_task_woken);
        portYIELD_FROM_ISR(high_task_woken);
    }
}

static void button_task(void *pvParameters) {
    gpio_num_t gpio_num;

    while (1) {
        if (!xQueueReceive(s_button_queue, &gpio_num, portMAX_DELAY)) {
            continue;
        }

        button_entry_t *btn = NULL;
        for (int i = 0; i < MAX_BUTTONS; i++) {
            if (s_buttons[i].is_registered && s_buttons[i].gpio_num == gpio_num) {
                btn = &s_buttons[i];
                break;
            }
        }

        if (btn == NULL) {
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - btn->last_press_tick) < pdMS_TO_TICKS(btn->debounce_ms)) {
            continue;
        }

        // Short wait to ensure mechanical contact settles
        vTaskDelay(pdMS_TO_TICKS(15));

        int current_level = gpio_get_level(btn->gpio_num);
        bool is_active = (current_level == (int)btn->active_level);

        if (is_active && btn->on_press_down != NULL) {
            btn->last_press_tick = xTaskGetTickCount();
            btn->on_press_down(btn->user_data);
        }
    }
}

esp_err_t button_init(const button_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid button configuration (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_GPIO(config->gpio_num)) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d", config->gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    // Find free or existing slot
    int slot = -1;
    for (int i = 0; i < MAX_BUTTONS; i++) {
        if (s_buttons[i].is_registered && s_buttons[i].gpio_num == config->gpio_num) {
            slot = i;
            break;
        } else if (!s_buttons[i].is_registered && slot == -1) {
            slot = i;
        }
    }

    if (slot == -1) {
        ESP_LOGE(TAG, "Maximum number of buttons (%d) reached", MAX_BUTTONS);
        return ESP_ERR_NO_MEM;
    }

    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (config->active_level == BUTTON_ACTIVE_LOW) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (config->active_level == BUTTON_ACTIVE_HIGH) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = (config->active_level == BUTTON_ACTIVE_LOW) ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", config->gpio_num, esp_err_to_name(ret));
        return ret;
    }

    // Create queue and task if not yet initialized
    if (s_button_queue == NULL) {
        s_button_queue = xQueueCreate(10, sizeof(gpio_num_t));
        if (s_button_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create button queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_button_task_handle == NULL) {
        BaseType_t task_ret = xTaskCreatePinnedToCore(
            button_task,
            "button_task",
            BUTTON_TASK_STACK_SIZE,
            NULL,
            BUTTON_TASK_PRIORITY,
            &s_button_task_handle,
            1
        );

        if (task_ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create button task");
            return ESP_ERR_NO_MEM;
        }
    }

    // Install ISR service if needed
    if (!s_isr_service_installed) {
        ret = gpio_install_isr_service(0);
        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
            s_isr_service_installed = true;
        } else {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Hook ISR handler
    ret = gpio_isr_handler_add(config->gpio_num, gpio_isr_handler, (void *)(intptr_t)config->gpio_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d: %s", config->gpio_num, esp_err_to_name(ret));
        return ret;
    }

    // Save button entry
    s_buttons[slot].gpio_num = config->gpio_num;
    s_buttons[slot].active_level = config->active_level;
    s_buttons[slot].debounce_ms = (config->debounce_ms > 0) ? config->debounce_ms : 50;
    s_buttons[slot].on_press_down = config->on_press_down;
    s_buttons[slot].user_data = config->user_data;
    s_buttons[slot].last_press_tick = 0;
    s_buttons[slot].is_registered = true;

    ESP_LOGI(TAG, "Button registered on GPIO %d (Active %s, Debounce %u ms)",
             config->gpio_num,
             (config->active_level == BUTTON_ACTIVE_LOW) ? "LOW" : "HIGH",
             (unsigned)s_buttons[slot].debounce_ms);

    return ESP_OK;
}

esp_err_t button_deinit(gpio_num_t gpio_num) {
    for (int i = 0; i < MAX_BUTTONS; i++) {
        if (s_buttons[i].is_registered && s_buttons[i].gpio_num == gpio_num) {
            gpio_isr_handler_remove(gpio_num);
            s_buttons[i].is_registered = false;
            ESP_LOGI(TAG, "Button unregistered on GPIO %d", gpio_num);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}
