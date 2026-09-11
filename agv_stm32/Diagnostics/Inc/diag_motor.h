/**
 * @file diag_motor.h
 * @brief Diagnostics & System Identification: Đo đáp ứng xung đơn vị (Unit Step Response) động cơ AGV
 * @details Xuất xung PWM 80% tới động cơ, đo tốc độ tức thời qua encoder và gửi dữ liệu qua RTT/UART
 */

#ifndef DIAG_MOTOR_H
#define DIAG_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include <stdint.h>
#include <stdbool.h>

/* Cấu hình tham số mặc định cho bài test chẩn đoán đáp ứng xung động cơ */
#define DIAG_MOTOR_DEFAULT_PWM           800     /**< 80% của 1000 (BSP_MOTOR_MAX_SPEED) */
#define DIAG_MOTOR_DEFAULT_DURATION_MS   2000    /**< Thời gian duy trì xung 80% (ms) */
#define DIAG_MOTOR_SAMPLE_RATE_MS        10      /**< Chu kỳ lấy mẫu 10ms (100Hz) */
#define DIAG_MOTOR_PRE_DELAY_MS          200     /**< Thời gian chờ baseline trước khi bật PWM (ms) */
#define DIAG_MOTOR_POST_DELAY_MS         500     /**< Thời gian quan sát quán tính giảm tốc sau khi ngắt PWM (ms) */
#define DIAG_MOTOR_CPR                   1320    /**< Số xung / vòng quay của Encoder AGV (4X mode) */
#define DIAG_MOTOR_FILTER_ALPHA          0.85f   /**< Hệ số bộ lọc thông thấp LPF/EMA cho vận tốc tức thời */

/**
 * @brief Trạng thái bài kiểm tra Unit Step Test động cơ
 */
typedef enum {
    DIAG_MOTOR_STATE_IDLE = 0,
    DIAG_MOTOR_STATE_RUNNING,
    DIAG_MOTOR_STATE_PASSED,
    DIAG_MOTOR_STATE_FAILED
} Diag_Motor_Status_t;

/**
 * @brief Kết quả đo đạc và đánh giá Unit Test động cơ
 */
typedef struct {
    Diag_Motor_Status_t status;        /**< Trạng thái đánh giá PASS/FAIL */
    Motor_ID_t motor;                  /**< Động cơ được test */
    int16_t target_pwm;                /**< PWM đích */
    uint32_t step_duration_ms;         /**< Thời gian duy trì xung (ms) */
    float peak_speed_rpm;              /**< Vận tốc cực đại ghi nhận (RPM) */
    float steady_speed_rpm;            /**< Vận tốc xác lập ước lượng (RPM) */
    int32_t total_encoder_ticks;       /**< Tổng xung encoder ghi nhận trong bài test */
    uint32_t total_samples;            /**< Tổng số mẫu dữ liệu */
    bool encoder_detected;             /**< Đã phát hiện xung encoder chuyển động */
    bool speed_reached;                /**< Vận tốc đã đạt ngưỡng tối thiểu */
} Diag_Motor_Result_t;

/**
 * @brief Khởi tạo module Step Response Test động cơ
 */
void Diag_Motor_Init(void);

/**
 * @brief Bắt đầu thực hiện bài kiểm tra đáp ứng xung đơn vị với cấu hình tùy chọn
 * @param motor Chọn động cơ kiểm tra (MOTOR_LEFT hoặc MOTOR_RIGHT)
 * @param target_pwm Giá trị xung PWM (-1000 đến +1000, 800 tương ứng 80%)
 * @param step_duration_ms Thời gian duy trì xung tính bằng ms (mặc định 2000ms)
 */
void Diag_Motor_Start(Motor_ID_t motor, int16_t target_pwm, uint32_t step_duration_ms);

/**
 * @brief Bắt đầu bài test với tham số mặc định: Motor Trái, 80% PWM (800/1000), 2000ms
 */
void Diag_Motor_StartDefault(void);

/**
 * @brief Dừng khẩn cấp bài kiểm tra và dừng động cơ
 */
void Diag_Motor_Stop(void);

/**
 * @brief Kiểm tra xem bài test có đang trong quá trình thực thi hay không
 * @return true nếu bài test đang chạy, false nếu ở trạng thái nghỉ
 */
bool Diag_Motor_IsRunning(void);

/**
 * @brief Lấy con trỏ struct kết quả của bài kiểm tra gần nhất
 * @return Con trỏ const tới struct kết quả Diag_Motor_Result_t
 */
const Diag_Motor_Result_t* Diag_Motor_GetLastResult(void);

/**
 * @brief Hàm xử lý chu kỳ lấy mẫu và điều khiển động cơ (Gọi định kỳ trong App_Loop)
 */
void Diag_Motor_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_MOTOR_H */
