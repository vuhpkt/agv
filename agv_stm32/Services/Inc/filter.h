#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Bộ lọc thông thấp bậc 1 (1st Order Low-Pass Filter / EMA)
 */
typedef struct {
    float alpha;             /**< Hệ số làm mượt [0.0 - 1.0], alpha càng nhỏ lọc càng mạnh */
    float prev_output;       /**< Giá trị đầu ra chu kỳ trước */
    bool is_initialized;     /**< Cờ đánh dấu đã nhận mẫu dữ liệu đầu tiên */
} filter_lpf_t;

/**
 * @brief Khởi tạo bộ lọc thông thấp (LPF)
 * @param filter Con trỏ tới filter handle
 * @param alpha Hệ số lọc (0.0f < alpha <= 1.0f)
 */
void filter_lpf_init(filter_lpf_t *filter, float alpha);

/**
 * @brief Cập nhật mẫu dữ liệu mới qua bộ lọc
 * @param filter Con trỏ tới filter handle
 * @param input Giá trị đo thô đầu vào
 * @return float Giá trị sau khi lọc
 */
float filter_lpf_update(filter_lpf_t *filter, float input);

/**
 * @brief Đặt lại trạng thái bộ lọc
 * @param filter Con trỏ tới filter handle
 */
void filter_lpf_reset(filter_lpf_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */

