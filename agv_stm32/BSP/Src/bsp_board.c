#include "bsp_board.h"
#include "main.h"

void BSP_LED_SetState(LED_State_t state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (state == LED_STATE_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BSP_LED_Toggle(void) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

uint32_t bsp_time_get_ms(void) {
    return HAL_GetTick();
}

void bsp_time_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
