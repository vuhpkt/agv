/**
 * @file app_motor_control.h
 * @brief Module điều khiển vận tốc kín (Closed-Loop Velocity Control) 2 bánh xe AGV
 * @details Sử dụng component motor_pi điều khiển TB6612 và đọc phản hồi từ Encoder
 */

#ifndef APP_MOTOR_CONTROL_H
#define APP_MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_pi.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Khởi tạo module điều khiển vận tốc 2 bánh xe
 */
void App_MotorControl_Init(void);

/**
 * @brief Đặt vận tốc mục tiêu (RPM) cho 2 bánh xe (Kích hoạt vòng kín)
 * @param left_rpm Tốc độ đặt bánh trái (RPM)
 * @param right_rpm Tốc độ đặt bánh phải (RPM)
 */
void App_MotorControl_SetTargetRPM(float left_rpm, float right_rpm);

/**
 * @brief Lấy tốc độ đo được thực tế hiện tại của 2 bánh xe
 */
void App_MotorControl_GetSpeeds(float *left_rpm, float *right_rpm);

/**
 * @brief Lấy tốc độ đặt mục tiêu hiện tại của 2 bánh xe
 */
void App_MotorControl_GetTargets(float *left_rpm, float *right_rpm);

/**
 * @brief Cập nhật hệ số Kp, Ki, Kff cho bộ điều khiển 2 bánh
 */
void App_MotorControl_SetGains(float kp, float ki, float k_ff);

/**
 * @brief Dừng điều khiển vòng kín và dừng động cơ
 */
void App_MotorControl_Stop(void);

/**
 * @brief Kiểm tra xem vòng kín điều khiển tốc độ có đang kích hoạt hay không
 */
bool App_MotorControl_IsEnabled(void);

/**
 * @brief In trạng thái chi tiết của bộ điều khiển PI ra RTT & UART
 */
void App_MotorControl_PrintStatus(void);

/**
 * @brief Hàm tính toán chu kỳ 10ms điều khiển động cơ (Gọi trong App_Loop)
 */
void App_MotorControl_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_CONTROL_H */


