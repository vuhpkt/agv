#include "bsp_ir.h"
#include "adc.h"
#include <stddef.h>
#include <stdbool.h>

/* Buffer DMA nhận dữ liệu liên tục từ ADC1 Scan Mode (PA2, PA3, PA4, PA5, PB0) */
static volatile uint16_t s_ir_raw[BSP_IR_NUM_SENSORS] = {0};
static bool s_initialized = false;

void BSP_IR_Init(void) {
    if (s_initialized) {
        return;
    }

    /* 1. Chạy hiệu chuẩn ADC1 trên STM32F1 để loại bỏ offset nội bộ */
    HAL_ADCEx_Calibration_Start(&hadc1);

    /* 2. Kích hoạt ADC1 quét liên tục và ghi vào mảng RAM qua DMA1 Channel 1 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_ir_raw, BSP_IR_NUM_SENSORS);

    s_initialized = true;
}

void BSP_IR_GetRaw(uint16_t out_raw[BSP_IR_NUM_SENSORS]) {
    if (out_raw == NULL) {
        return;
    }
    for (int i = 0; i < BSP_IR_NUM_SENSORS; i++) {
        out_raw[i] = s_ir_raw[i];
    }
}
