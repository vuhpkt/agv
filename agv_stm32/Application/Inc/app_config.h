/**
 * @file app_config.h
 * @brief Cấu hình chế độ vận hành (Operational Mode) của xe tự hành AGV
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bật/tắt driver giao tiếp UART TTL (USART1)
 * - 0: [Mặc định] Tắt hoàn toàn giao tiếp UART TTL, giải phóng >1.3KB RAM
 *      và giải phóng tài nguyên DMA1 Channel 4/5. Toàn bộ log và giao tiếp
 *      chuyển sang cổng SEGGER RTT tốc độ cao qua ST-Link.
 * - 1: Bật driver UART TTL DMA với hàng đợi RingBuffer.
 */
#define AGV_ENABLE_UART_TTL             1

/**
 * @brief Chế độ kiểm thử bàn test độc lập (Bench Unit-Test Mode)
 * - 0: [Vận hành AGV thực tế]
 * - 1: [Chế độ kiểm thử bàn test / Diagnostics qua UART với ESP32]
 */
#define AGV_ENABLE_MOTOR_TEST_MODE      1

/**
 * @brief Tự động khởi động bám line sau khi bật nguồn (Auto-Start trên sa bàn)
 * - 0: [Mặc định] Không tự chạy khi khởi động (chờ lệnh từ ESP32 qua UART).
 * - 1: Tự động đếm ngược AGV_LINE_FOLLOW_ARM_DELAY_MS (3 giây), nháy nhanh LED PC13
 *      để người dùng đặt xe ngay ngắn xuống vạch rồi tự động kích hoạt bám line độc lập.
 */
#define AGV_LINE_FOLLOW_AUTO_START      0
#define AGV_LINE_FOLLOW_DEFAULT_RPM     80.0f
#define AGV_LINE_FOLLOW_ARM_DELAY_MS    3000

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
