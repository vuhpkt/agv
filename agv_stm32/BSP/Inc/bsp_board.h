#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LED_STATE_ON,
    LED_STATE_OFF
} LED_State_t;

void BSP_LED_SetState(LED_State_t state);
void BSP_LED_Toggle(void);

uint32_t bsp_time_get_ms(void);
void bsp_time_delay_ms(uint32_t ms);

#endif

