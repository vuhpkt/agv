#ifndef BSP_IR_H
#define BSP_IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define BSP_IR_NUM_SENSORS 5

/**
 * @brief Khởi tạo phần cứng ADC1 DMA và chạy hiệu chuẩn cho 5 mắt IR
 */
void BSP_IR_Init(void);

/**
 * @brief Đọc giá trị raw tức thời (0-4095) của 5 mắt IR từ DMA buffer (O(1), non-blocking)
 * @param out_raw Mảng 5 phần tử chứa giá trị đọc được (Rank 1..5: PA2, PA3, PA4, PA5, PB0)
 */
void BSP_IR_GetRaw(uint16_t out_raw[BSP_IR_NUM_SENSORS]);

#ifdef __cplusplus
}
#endif

#endif /* BSP_IR_H */
