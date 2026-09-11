#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_callback_t)(void *user_data);

typedef enum {
    BUTTON_ACTIVE_LOW = 0,   // Pulled high, pressed goes to GND (e.g. BOOT button)
    BUTTON_ACTIVE_HIGH = 1,  // Pulled low, pressed goes to VDD
} button_active_level_t;

typedef struct {
    gpio_num_t gpio_num;
    button_active_level_t active_level;
    uint32_t debounce_ms;               // Debounce time in ms (default: 50ms)
    button_callback_t on_press_down;    // Callback invoked when button is pressed down
    void *user_data;                    // Optional context pointer passed to callback
} button_config_t;

#define BUTTON_CONFIG_DEFAULT(gpio) {  \
    .gpio_num = (gpio),                 \
    .active_level = BUTTON_ACTIVE_LOW,  \
    .debounce_ms = 50,                  \
    .on_press_down = NULL,              \
    .user_data = NULL                   \
}

/**
 * @brief Initialize a button with GPIO interrupt, debounce filtering, and callback.
 * 
 * @param config Pointer to button_config_t structure.
 * @return esp_err_t ESP_OK on success, or error code.
 */
esp_err_t button_init(const button_config_t *config);

/**
 * @brief De-initialize the button and free its resources.
 * 
 * @param gpio_num The GPIO number of the button.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t button_deinit(gpio_num_t gpio_num);

#ifdef __cplusplus
}
#endif

