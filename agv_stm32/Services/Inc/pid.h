#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Cấu hình tham số PID Controller
 */
typedef struct {
    float kp;                /**< Hệ số khuếch đại tỉ lệ */
    float ki;                /**< Hệ số tích phân */
    float kd;                /**< Hệ số vi phân */
    float out_min;           /**< Giới hạn bão hòa ngõ ra cực tiểu */
    float out_max;           /**< Giới hạn bão hòa ngõ ra cực đại */
    float integral_limit;    /**< Giới hạn chống bão hòa tích phân (Anti-windup limit) */
} pid_cfg_t;

/**
 * @brief Handle quản lý trạng thái runtime của bộ điều khiển PID
 */
typedef struct {
    pid_cfg_t cfg;           /**< Cấu hình tham số */
    float prev_error;        /**< Sai số ở chu kỳ trước */
    float integral;          /**< Thành phần tích phân tích lũy */
    float output;            /**< Giá trị đầu ra tính toán gần nhất */
} pid_t;

/**
 * @brief Khởi tạo bộ điều khiển PID
 * @param pid Con trỏ tới PID handle
 * @param cfg Con trỏ tới struct tham số cấu hình
 */
void pid_init(pid_t *pid, const pid_cfg_t *cfg);

/**
 * @brief Tính toán ngõ ra điều khiển PID
 * @param pid Con trỏ tới PID handle
 * @param setpoint Giá trị đặt (Target)
 * @param measurement Giá trị đo được thực tế (Feedback)
 * @param dt Chu kỳ lấy mẫu (đơn vị: giây)
 * @return float Giá trị điều khiển ngõ ra (đã bão hòa trong [out_min, out_max])
 */
float pid_calculate(pid_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief Đặt lại trạng thái tích phân và sai số của PID về 0
 * @param pid Con trỏ tới PID handle
 */
void pid_reset(pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */

