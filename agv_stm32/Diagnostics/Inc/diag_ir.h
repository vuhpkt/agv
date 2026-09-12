/**
 * @file diag_ir.h
 * @brief Unit test & Diagnostics: Kiểm tra độc lập chức năng dò line của mảng cảm biến 5 mắt IR
 * @details Module chẩn đoán độc lập cho mảng IR:
 *          đo raw ADC, kiểm tra dải động, hiệu chuẩn min/max, và chạy thử nghiệm bám line.
 */

#ifndef DIAG_IR_H
#define DIAG_IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_ir.h"
#include "line_tracker.h"
#include <stdint.h>
#include <stdbool.h>

#define DIAG_IR_DEFAULT_DURATION_MS    3000    /**< Thời gian chạy test mặc định (ms) */
#define DIAG_IR_SAMPLE_RATE_MS         10      /**< Chu kỳ lấy mẫu 10ms (100Hz) */
#define DIAG_IR_DEFAULT_BASE_RPM       80.0f   /**< Vận tốc cơ sở khi test chạy bám line */
#define DIAG_IR_MIN_DYNAMIC_RANGE      100     /**< Ngưỡng dải động tối thiểu để coi mắt IR hoạt động */
#define DIAG_IR_LOST_TIMEOUT_MS        2000    /**< Thời gian ngắt khẩn cấp khi mất line liên tục (2.0 giây) */

/**
 * @brief Chế độ kiểm tra chẩn đoán dò line của mảng IR
 */
typedef enum {
    DIAG_IR_MODE_SENSOR = 0,   /**< Test giám sát: Đọc và stream 5 mắt IR, kiểm tra cảm biến sống/chết */
    DIAG_IR_MODE_CALIB,        /**< Test hiệu chuẩn: Quét và tự động lưu min/max quang học */
    DIAG_IR_MODE_TRACK         /**< Test động: Chạy bám line vòng kín có giới hạn thời gian */
} Diag_IR_Mode_t;

/**
 * @brief Trạng thái bài kiểm tra IR Line Test
 */
typedef enum {
    DIAG_IR_STATE_IDLE = 0,
    DIAG_IR_STATE_RUNNING,
    DIAG_IR_STATE_PASSED,
    DIAG_IR_STATE_FAILED
} Diag_IR_Status_t;

/**
 * @brief Kết quả đo đạc và đánh giá Unit Test mảng IR
 */
typedef struct {
    Diag_IR_Status_t status;               /**< Trạng thái đánh giá PASS/FAIL */
    Diag_IR_Mode_t mode;                   /**< Chế độ test vừa thực hiện */
    uint32_t duration_ms;                  /**< Thời gian thực hiện test (ms) */
    uint32_t total_samples;                /**< Tổng số mẫu dữ liệu thu thập */
    uint16_t min_raw[BSP_IR_NUM_SENSORS];  /**< Giá trị raw nhỏ nhất ghi nhận */
    uint16_t max_raw[BSP_IR_NUM_SENSORS];  /**< Giá trị raw lớn nhất ghi nhận */
    int16_t last_error;                    /**< Sai số tâm e_line cuối cùng */
    bool all_sensors_responding;           /**< Tất cả 5 cảm biến đều phản hồi tín hiệu hợp lệ */
    bool line_detected;                    /**< Đã phát hiện vạch line hợp lệ */
    bool crossline_detected;               /**< Đã phát hiện vạch ngang ngã tư */
} Diag_IR_Result_t;

/**
 * @brief Khởi tạo module kiểm tra dò line
 */
void Diag_IR_Init(void);

/**
 * @brief Bắt đầu bài kiểm tra dò line với chế độ và thời lượng tùy chọn
 * @param mode Chế độ test (DIAG_IR_MODE_SENSOR, DIAG_IR_MODE_CALIB, DIAG_IR_MODE_TRACK)
 * @param duration_ms Thời gian chạy test tính bằng ms (0 = mặc định 3000ms)
 * @param base_rpm Vận tốc cơ sở cho chế độ TRACK (0 = mặc định 80 RPM)
 */
void Diag_IR_Start(Diag_IR_Mode_t mode, uint32_t duration_ms, float base_rpm);

/**
 * @brief Bắt đầu bài test với tham số mặc định (giám sát mảng IR trong 3000ms)
 */
void Diag_IR_StartDefault(void);

/**
 * @brief Dừng khẩn cấp bài kiểm tra và dừng động cơ
 */
void Diag_IR_Stop(void);

/**
 * @brief Kiểm tra xem bài test có đang chạy hay không
 * @return true nếu đang chạy, false nếu ở trạng thái nghỉ
 */
bool Diag_IR_IsRunning(void);

/**
 * @brief Lấy con trỏ tới kết quả bài test gần nhất
 * @return Con trỏ const tới struct kết quả Diag_IR_Result_t
 */
const Diag_IR_Result_t* Diag_IR_GetLastResult(void);

/**
 * @brief Hàm xử lý chu kỳ lấy mẫu và điều khiển bài test (Gọi trong vòng lặp)
 */
void Diag_IR_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_IR_H */
