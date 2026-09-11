#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Callback function prototypes for Hardware Encoder Timer Binding
 */
typedef uint32_t (*encoder_get_raw_fn)(void *user_data);
typedef void (*encoder_reset_raw_fn)(void *user_data);

/**
 * @brief Hardware I/O Interface for Encoder
 */
typedef struct {
    encoder_get_raw_fn get_raw_count;     /**< Callback đọc giá trị thanh ghi counter của Timer */
    encoder_reset_raw_fn reset_raw_count; /**< Callback reset thanh ghi counter về 0 (tùy chọn) */
} encoder_io_t;

/**
 * @brief Static configuration for Encoder channel
 */
typedef struct {
    bool inverted;               /**< Đảo chiều đọc xung logic */
    uint16_t cpr;                /**< Số xung trên một vòng quay (Counts Per Revolution - tùy chọn) */
} encoder_cfg_t;

/**
 * @brief Encoder Channel Handle (Runtime State)
 */
typedef struct {
    encoder_cfg_t cfg;
    encoder_io_t io;
    void *user_data;             /**< Con trỏ dữ liệu tùy chọn truyền vào callbacks */
    uint16_t last_raw_cnt;       /**< Giá trị đếm raw ở chu kỳ trước */
    int16_t last_delta;          /**< Số xung thay đổi trong chu kỳ vừa qua */
    int32_t total_count;         /**< Tổng số xung tích lũy 32-bit */
} encoder_t;

/**
 * @brief Khởi tạo handle bộ đọc xung Encoder
 * @param handle Con trỏ tới encoder handle
 * @param cfg Con trỏ tới struct cấu hình (nếu NULL sẽ dùng cấu hình mặc định)
 * @param io Con trỏ tới struct callbacks IO phần cứng
 * @param user_data Con trỏ dữ liệu tùy chọn truyền vào callbacks
 */
void encoder_init(encoder_t *handle, const encoder_cfg_t *cfg, const encoder_io_t *io, void *user_data);

/**
 * @brief Cập nhật tính toán xung (Gọi định kỳ trong vòng lặp hoặc timer interrupt)
 * @param handle Con trỏ tới encoder handle
 */
void encoder_update(encoder_t *handle);

/**
 * @brief Lấy tổng số xung tích lũy 32-bit
 * @param handle Con trỏ tới encoder handle
 * @return int32_t Tổng số xung (có dấu)
 */
int32_t encoder_get_count(const encoder_t *handle);

/**
 * @brief Lấy số xung thay đổi (delta) ở chu kỳ cập nhật gần nhất
 * @param handle Con trỏ tới encoder handle
 * @return int16_t Số xung delta (có dấu)
 */
int16_t encoder_get_delta(const encoder_t *handle);

/**
 * @brief Đặt lại giá trị đếm và delta về 0
 * @param handle Con trỏ tới encoder handle
 */
void encoder_reset(encoder_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */

