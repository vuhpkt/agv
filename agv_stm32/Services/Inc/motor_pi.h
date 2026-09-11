/**
 * @file motor_pi.h
 * @brief Component Bộ điều khiển vận tốc Motor PI (Velocity Closed-Loop Controller) cho AGV
 * @details Kết hợp mô hình 2-DOF (Feedforward + PI phản hồi), Anti-Windup Clamping,
 *          bù vùng chết ma sát tĩnh (Deadzone) và giới hạn dốc gia tốc (Slew Rate Limiter).
 *          Các thông số mặc định được tối ưu trực tiếp từ nhận dạng hàm truyền thực tế:
 *          G(s) = 0.2699 / (0.0449s + 1).
 */

#ifndef MOTOR_PI_H
#define MOTOR_PI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Tham số nhận dạng thực nghiệm của động cơ AGV (Từ bài test Unit Step 80% PWM)
 */
#define MOTOR_PI_PLANT_K            0.2625f   /**< Độ lợi tĩnh K (RPM / PWM unit) */
#define MOTOR_PI_PLANT_TAU          0.0467f   /**< Hằng số thời gian tau (giây) */

/**
 * @brief Tham số điều khiển mặc định tối ưu theo phương pháp gán cực (Pole Placement)
 */
#define MOTOR_PI_DEFAULT_KP         5.34f     /**< Hệ số khuếch đại tỉ lệ Kp */
#define MOTOR_PI_DEFAULT_KI         114.3f    /**< Hệ số khuếch đại tích phân Ki (Ki = Kp / tau) */
#define MOTOR_PI_DEFAULT_KFF        3.810f    /**< Hệ số Feedforward Kff = 1 / K */
#define MOTOR_PI_DEFAULT_OUT_MAX    1000.0f   /**< Giới hạn PWM bão hòa cực đại (+100%) */
#define MOTOR_PI_DEFAULT_OUT_MIN    -1000.0f  /**< Giới hạn PWM bão hòa cực tiểu (-100%) */
#define MOTOR_PI_DEFAULT_I_MAX      400.0f    /**< Giới hạn tích phân chống bão hòa (Anti-Windup) */
#define MOTOR_PI_DEFAULT_DEADZONE   40.0f     /**< Bù vùng chết ma sát tĩnh (PWM unit) */
#define MOTOR_PI_DEFAULT_ACCEL_LIM  600.0f    /**< Giới hạn gia tốc dốc (RPM/s, tránh giật dòng) */

/**
 * @brief Cấu hình tham số bộ điều khiển vận tốc Motor PI
 */
typedef struct {
    float kp;                 /**< Hệ số khuếch đại tỉ lệ Kp */
    float ki;                 /**< Hệ số khuếch đại tích phân Ki */
    float k_ff;               /**< Hệ số khuếch đại Feedforward Kff */
    float out_min;            /**< Giới hạn ngõ ra tối thiểu (PWM) */
    float out_max;            /**< Giới hạn ngõ ra tối đa (PWM) */
    float integral_limit;     /**< Ngưỡng chống tràn tích phân Anti-Windup */
    float deadzone_pwm;       /**< Bù điện áp khởi động ma sát tĩnh */
    float max_accel_rpm_s;    /**< Giới hạn gia tốc dốc (RPM/s, 0 = không giới hạn) */
} motor_pi_cfg_t;

/**
 * @brief Handle quản lý trạng thái runtime của bộ điều khiển Motor PI
 */
typedef struct {
    motor_pi_cfg_t cfg;       /**< Bảng tham số cấu hình */
    float target_rpm;         /**< Tốc độ đặt tức thời (sau khi qua bộ giới hạn gia tốc) */
    float measured_rpm;       /**< Tốc độ đo được từ encoder */
    float error;              /**< Sai số vận tốc e = target - measured */
    float prev_error;         /**< Sai số ở chu kỳ trước */
    float integral;           /**< Giá trị tích phân tích lũy */
    float p_term;             /**< Đóng góp thành phần P gần nhất */
    float i_term;             /**< Đóng góp thành phần I gần nhất */
    float ff_term;            /**< Đóng góp thành phần Feedforward gần nhất */
    float output_pwm;         /**< Tín hiệu điều khiển PWM ngõ ra [-out_max, out_max] */
    bool is_saturated;        /**< Cờ báo ngõ ra đang bị bão hòa kịch kim */
} motor_pi_t;

/**
 * @brief Khởi tạo bộ điều khiển Motor PI với cấu hình tùy chọn
 * @param ctrl Con trỏ tới motor_pi handle
 * @param cfg Con trỏ cấu hình (nếu NULL sẽ dùng cấu hình mặc định tối ưu)
 */
void motor_pi_init(motor_pi_t *ctrl, const motor_pi_cfg_t *cfg);

/**
 * @brief Lấy cấu hình mặc định tối ưu được tính toán từ mô hình nhận dạng thực tế
 * @param cfg Con trỏ struct cấu hình để ghi dữ liệu vào
 */
void motor_pi_get_default_cfg(motor_pi_cfg_t *cfg);

/**
 * @brief Tự động tính toán bộ thông số PI tối ưu từ kết quả nhận dạng FOPTD
 * @param cfg Con trỏ cấu hình đầu ra
 * @param k Độ lợi tĩnh K (RPM/PWM)
 * @param tau_s Hằng số thời gian tau (giây)
 * @param desired_ts_s Thời gian xác lập mong muốn của hệ kín (giây, ví dụ 0.10s)
 */
void motor_pi_tune_from_plant(motor_pi_cfg_t *cfg, float k, float tau_s, float desired_ts_s);

/**
 * @brief Tính toán chu kỳ điều khiển vận tốc (gọi định kỳ trong loop điều khiển, ví dụ 10ms / 100Hz)
 * @param ctrl Con trỏ tới motor_pi handle
 * @param setpoint_rpm Tốc độ đặt mục tiêu (RPM)
 * @param feedback_rpm Tốc độ đo thực tế (RPM, từ encoder)
 * @param dt_s Chu kỳ lấy mẫu (giây, ví dụ 0.010f cho 10ms)
 * @return int16_t Giá trị xung PWM điều khiển động cơ [-1000, 1000]
 */
int16_t motor_pi_update(motor_pi_t *ctrl, float setpoint_rpm, float feedback_rpm, float dt_s);

/**
 * @brief Đặt lại trạng thái tích phân, sai số và đầu ra về 0
 * @param ctrl Con trỏ tới motor_pi handle
 */
void motor_pi_reset(motor_pi_t *ctrl);

/**
 * @brief Cập nhật nhanh các hệ số Kp, Ki, Kff lúc runtime
 * @param ctrl Con trỏ tới motor_pi handle
 * @param kp Hệ số P mới
 * @param ki Hệ số I mới
 * @param k_ff Hệ số Feedforward mới
 */
void motor_pi_set_gains(motor_pi_t *ctrl, float kp, float ki, float k_ff);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PI_H */

