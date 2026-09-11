#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "line_tracker.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LINE_STATE_IDLE = 0,
    LINE_STATE_CALIBRATING,
    LINE_STATE_TRACKING,
    LINE_STATE_CROSS_LINE,
    LINE_STATE_TURNING_LEFT,
    LINE_STATE_TURNING_RIGHT,
    LINE_STATE_LOST
} line_state_t;

/**
 * @brief Khởi tạo module điều khiển bám line
 */
void App_LineFollow_Init(void);

/**
 * @brief Bắt đầu bám line đơn thuần với vận tốc cơ sở base_rpm
 */
void App_LineFollow_Start(float base_rpm);

/**
 * @brief Dừng bám line và dừng động cơ
 */
void App_LineFollow_Stop(void);

/**
 * @brief Nạp chuỗi lộ trình FSM cho AGV
 * @param actions Mảng các hành vi tại từng ngã rẽ
 * @param count Số lượng bước
 * @param target_rpm Vận tốc chạy danh định
 * @return true nếu nạp thành công
 */
bool App_LineFollow_LoadRoute(const uint8_t *actions, uint8_t count, float target_rpm);

/**
 * @brief Kích hoạt bắt đầu chạy chuỗi lộ trình đã nạp
 */
void App_LineFollow_StartRoute(void);

/**
 * @brief Hủy chuỗi lộ trình và dừng khẩn cấp
 */
void App_LineFollow_CancelRoute(void);

/**
 * @brief Lấy chỉ số bước hiện tại trong lộ trình
 */
uint8_t App_LineFollow_GetCurrentStep(void);

/**
 * @brief Kiểm tra xem đang chạy lộ trình tự hành hay không
 */
bool App_LineFollow_IsRouteRunning(void);

/**
 * @brief Bắt đầu chế độ tự hiệu chuẩn cảm biến (chạy trong 3 giây)
 */
void App_LineFollow_StartCalibration(void);

/**
 * @brief Xử lý chu kỳ bám line (gọi định kỳ từ App_Loop)
 */
void App_LineFollow_Process(void);

/**
 * @brief Lấy trạng thái hiện tại của bộ bám line
 */
line_state_t App_LineFollow_GetState(void);

/**
 * @brief Kiểm tra xem có đang ở nút giao/vạch ngang hay không
 */
bool App_LineFollow_IsCrossline(void);

/**
 * @brief Lấy giá trị sai số lệch line gần nhất
 */
int16_t App_LineFollow_GetLastError(void);

/**
 * @brief Đọc thông tin cảm biến phục vụ debug / RTT CLI
 */
void App_LineFollow_GetSensors(uint16_t raw[LINE_TRACKER_NUM_SENSORS], uint16_t norm[LINE_TRACKER_NUM_SENSORS], int16_t *error);

/**
 * @brief Cập nhật tham số Line PID
 */
void App_LineFollow_SetPID(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_FOLLOW_H */
