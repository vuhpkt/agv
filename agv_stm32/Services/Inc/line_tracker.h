#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LINE_TRACKER_NUM_SENSORS 5

/**
 * @brief Trọng số hình học mảng 5 mắt IR (khoảng cách thực tế tính từ mắt tâm):
 * - Mắt 3 (Tâm): 0.00 cm -> 0
 * - Mắt 2 & 4:   1.25 cm -> 385  (1000 * 1.25 / 3.25)
 * - Mắt 1 & 5:   3.25 cm -> 1000 (1000 * 3.25 / 3.25)
 */
#define LINE_TRACKER_WEIGHT_OUTER   1000
#define LINE_TRACKER_WEIGHT_INNER   385
#define LINE_TRACKER_WEIGHT_CENTER  0

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_max;           /**< Giới hạn tối đa của lượng bẻ lái delta_rpm */
} line_pid_cfg_t;

typedef struct {
    uint16_t min_raw[LINE_TRACKER_NUM_SENSORS];  /**< Giá trị ngưỡng cực tiểu (đen hoặc trắng) */
    uint16_t max_raw[LINE_TRACKER_NUM_SENSORS];  /**< Giá trị ngưỡng cực đại */
    uint16_t threshold[LINE_TRACKER_NUM_SENSORS];/**< Ngưỡng phân biệt 0/1 */
    bool is_white_line;                          /**< true nếu vạch trắng nền đen, false nếu vạch đen nền trắng */
} line_calib_t;

typedef struct {
    line_calib_t calib;
    line_pid_cfg_t pid_cfg;
    float integral;
    float prev_error;
    int16_t last_valid_error;
    bool line_lost;
    bool is_crossline;
} line_tracker_t;

/**
 * @brief Khởi tạo handle bộ bám line
 */
void line_tracker_init(line_tracker_t *tracker, const line_pid_cfg_t *pid_cfg);

/**
 * @brief Thiết lập cấu hình hiệu chuẩn trắng/đen
 */
void line_tracker_set_calib(line_tracker_t *tracker, const line_calib_t *calib);

/**
 * @brief Cập nhật liên tục giá trị min/max từ mẫu raw đọc được (dùng khi chạy quét calib)
 */
void line_tracker_update_calib_minmax(line_tracker_t *tracker, const uint16_t raw[LINE_TRACKER_NUM_SENSORS]);

/**
 * @brief Chuẩn hóa giá trị 5 cảm biến về dải [0 .. 1000] (1000 = tâm vạch)
 */
void line_tracker_normalize(const line_tracker_t *tracker, const uint16_t raw[LINE_TRACKER_NUM_SENSORS], uint16_t out_norm[LINE_TRACKER_NUM_SENSORS]);

/**
 * @brief Tính sai số vị trí vạch [-1000 .. +1000] theo phương pháp trọng số
 * @return int16_t: 0 khi ở tâm, âm khi lệch phải, dương khi lệch trái
 */
int16_t line_tracker_compute_error(line_tracker_t *tracker, const uint16_t norm[LINE_TRACKER_NUM_SENSORS]);

/**
 * @brief Tính toán lượng bù vận tốc góc delta_rpm dựa trên Line PID
 */
float line_tracker_compute_pid(line_tracker_t *tracker, int16_t error, float dt_s);

/**
 * @brief Reset bộ tích phân và trạng thái lỗi của PID
 */
void line_tracker_reset(line_tracker_t *tracker);

#ifdef __cplusplus
}
#endif

#endif /* LINE_TRACKER_H */
