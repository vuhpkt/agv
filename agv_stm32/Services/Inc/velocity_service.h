/**
 * @file velocity_service.h
 * @brief Dịch vụ điều khiển vận tốc bánh xe xe tự hành AGV (Tầng Services - Pure C / Zero-HAL)
 * @details Đóng gói công thức CPR -> RPM, bộ lọc thông thấp LPF/EMA, và bộ điều khiển kín Motor PI.
 */

#ifndef VELOCITY_SERVICE_H
#define VELOCITY_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_pi.h"
#include "filter.h"

/**
 * @brief Cấu hình dịch vụ điều khiển vận tốc cho 1 kênh bánh xe
 */
typedef struct {
    uint16_t cpr;              /**< Số xung trên 1 vòng quay encoder (ví dụ: 1320) */
    float filter_alpha;        /**< Hệ số lọc thông thấp (ví dụ: 0.85f) */
    motor_pi_cfg_t pi_cfg;     /**< Bảng thông số bộ điều khiển PI */
} velocity_cfg_t;

/**
 * @brief Handle quản lý trạng thái vận tốc của 1 kênh bánh xe
 */
typedef struct {
    velocity_cfg_t cfg;
    motor_pi_t pi;
    filter_lpf_t filter;

    float target_rpm;          /**< Tốc độ đặt mục tiêu (RPM) */
    float raw_rpm;             /**< Tốc độ thô đo từ delta xung (RPM) */
    float filtered_rpm;        /**< Tốc độ sau lọc LPF (RPM) */
    int16_t output_pwm;        /**< Tín hiệu PWM ngõ ra [-1000, 1000] */
    bool is_enabled;           /**< Trạng thái kích hoạt vòng kín */
} velocity_channel_t;

/**
 * @brief Chuyển đổi delta encoder ticks sang vận tốc vòng/phút (RPM)
 * @param cpr Số xung trên một vòng quay của encoder
 * @param delta_ticks Số xung encoder đếm được trong chu kỳ dt
 * @param dt_s Chu kỳ lấy mẫu tính bằng giây
 * @return float Vận tốc tức thời (RPM)
 */
float velocity_service_ticks_to_rpm(uint16_t cpr, int16_t delta_ticks, float dt_s);

/**
 * @brief Khởi tạo kênh điều khiển vận tốc
 * @param ch Con trỏ tới velocity_channel handle
 * @param cpr Số xung trên 1 vòng quay encoder (CPR)
 * @param filter_alpha Hệ số lọc thông thấp LPF (0.0 đến 1.0)
 * @param pi_cfg Con trỏ cấu hình PI (nếu NULL sẽ dùng cấu hình mặc định tối ưu)
 */
void velocity_service_init(velocity_channel_t *ch, uint16_t cpr, float filter_alpha, const motor_pi_cfg_t *pi_cfg);

/**
 * @brief Thiết lập tốc độ đặt mục tiêu (RPM) và bật chế độ điều khiển vòng kín
 * @param ch Con trỏ tới velocity_channel handle
 * @param target_rpm Vận tốc đặt (RPM)
 */
void velocity_service_set_target(velocity_channel_t *ch, float target_rpm);

/**
 * @brief Cập nhật chu kỳ điều khiển vận tốc: Đổi delta xung -> Raw RPM -> LPF -> Motor PI -> PWM
 * @param ch Con trỏ tới velocity_channel handle
 * @param delta_ticks Số xung encoder thay đổi trong chu kỳ vừa qua
 * @param dt_s Chu kỳ lấy mẫu (giây, ví dụ 0.010f cho 10ms)
 * @return int16_t Giá trị PWM điều khiển động cơ [-1000, 1000]
 */
int16_t velocity_service_update(velocity_channel_t *ch, int16_t delta_ticks, float dt_s);

/**
 * @brief Reset toàn bộ trạng thái tích phân, sai số và đầu ra về 0
 * @param ch Con trỏ tới velocity_channel handle
 */
void velocity_service_reset(velocity_channel_t *ch);

/**
 * @brief Cập nhật hệ số Kp, Ki, Kff cho kênh
 * @param ch Con trỏ tới velocity_channel handle
 * @param kp Hệ số P mới
 * @param ki Hệ số I mới
 * @param k_ff Hệ số Feedforward mới
 */
void velocity_service_set_gains(velocity_channel_t *ch, float kp, float ki, float k_ff);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_SERVICE_H */
