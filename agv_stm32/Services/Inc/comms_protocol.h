/**
 * @file comms_protocol.h
 * @brief Giao thức truyền thông nhị phân AGV MCU-to-MCU (Pure C / Zero-HAL)
 * Dùng chung 100% giữa STM32 và ESP32.
 */

#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define AGV_PROTOCOL_SOF1            0xAA
#define AGV_PROTOCOL_SOF2            0x55
#define AGV_PROTOCOL_MAX_PAYLOAD     128
#define AGV_PROTOCOL_HEADER_SIZE     4   /* SOF1 + SOF2 + MSG_ID + LEN */
#define AGV_PROTOCOL_CRC_SIZE        2
#define AGV_PROTOCOL_MAX_FRAME_SIZE  (AGV_PROTOCOL_HEADER_SIZE + AGV_PROTOCOL_MAX_PAYLOAD + AGV_PROTOCOL_CRC_SIZE)
#define AGV_ROUTE_MAX_ACTIONS        16

/* ========================================================================= */
/* Mã Định Danh Thông Điệp (Message IDs)                                     */
/* ========================================================================= */
typedef enum {
    /* Nhóm Hệ Thống & Lệnh Cơ Bản */
    AGV_MSG_PING             = 0x01,
    AGV_MSG_EMERGENCY_STOP   = 0x02,

    /* Nhóm Điều Khiển Lộ Trình (Route) */
    AGV_MSG_ROUTE_LOAD       = 0x10,
    AGV_MSG_ROUTE_START      = 0x11,
    AGV_MSG_ROUTE_CANCEL     = 0x12,

    /* Nhóm Chẩn Đoán & Kiểm Thử Phần Cứng (Diag) */
    AGV_MSG_DIAG_STEP_TEST   = 0x20,
    AGV_MSG_DIAG_SET_SPEED   = 0x21,
    AGV_MSG_DIAG_SET_RPM     = 0x22,
    AGV_MSG_DIAG_GET_ENC     = 0x23,
    AGV_MSG_DIAG_RESET_ENC   = 0x24,
    AGV_MSG_DIAG_PI_TUNE     = 0x25,
    AGV_MSG_DIAG_LINE_TEST   = 0x26,

    /* Phản Hồi & Báo Cáo (Telemetry / Events) */
    AGV_MSG_ACK              = 0x80,
    AGV_MSG_PONG             = 0x81,
    AGV_MSG_MISSION_STATUS   = 0x90,
    AGV_MSG_TELEMETRY        = 0x91,
    AGV_MSG_LOG              = 0x99,
    AGV_MSG_DIAG_ENC_DATA    = 0xA3
} agv_msg_id_t;

/* ========================================================================= */
/* Enums Trạng Thái & Hành Vi                                                */
/* ========================================================================= */
typedef enum {
    AGV_ACTION_STOP          = 0x00,
    AGV_ACTION_FORWARD       = 0x01,
    AGV_ACTION_TURN_LEFT     = 0x02,
    AGV_ACTION_TURN_RIGHT    = 0x03
} agv_route_action_t;

typedef enum {
    AGV_MISSION_IDLE         = 0x00,
    AGV_MISSION_RUNNING      = 0x01,
    AGV_MISSION_COMPLETED    = 0x02,
    AGV_MISSION_ERROR_LOST   = 0x03,
    AGV_MISSION_ERROR_TIMEOUT= 0x04
} agv_mission_state_t;

typedef enum {
    AGV_ACK_OK               = 0x00,
    AGV_ACK_ERR_CRC          = 0x01,
    AGV_ACK_ERR_UNKNOWN_CMD  = 0x02,
    AGV_ACK_ERR_INVALID_ARGS = 0x03,
    AGV_ACK_ERR_BUSY         = 0x04
} agv_ack_status_t;

/* ========================================================================= */
/* Cấu Trúc Dữ Liệu Payload (Packed Structs)                                 */
/* ========================================================================= */
#pragma pack(push, 1)

typedef struct {
    uint8_t count;                          /**< Số bước trong lộ trình (tối đa 16) */
    int16_t target_rpm_x10;                 /**< Vận tốc cơ sở (RPM * 10) */
    uint8_t actions[AGV_ROUTE_MAX_ACTIONS]; /**< Danh sách hành vi tại ngã rẽ */
} agv_payload_route_load_t;

typedef struct {
    uint8_t motor_id;                       /**< 0: Trái, 1: Phải */
    int16_t pwm;                            /**< Mức PWM (-1000 đến 1000) */
    uint32_t duration_ms;                   /**< Thời gian chạy (ms) */
} agv_payload_diag_step_test_t;

typedef struct {
    int16_t left_pwm;                       /**< PWM motor trái (-1000..1000) */
    int16_t right_pwm;                      /**< PWM motor phải (-1000..1000) */
} agv_payload_diag_set_speed_t;

typedef struct {
    int16_t left_rpm_x10;                   /**< RPM motor trái * 10 */
    int16_t right_rpm_x10;                  /**< RPM motor phải * 10 */
} agv_payload_diag_set_rpm_t;

typedef struct {
    int32_t enc_left;                       /**< Xung encoder bánh trái */
    int32_t enc_right;                      /**< Xung encoder bánh phải */
} agv_payload_diag_enc_data_t;

typedef struct {
    uint16_t kp_x100;                       /**< Kp * 100 */
    uint16_t ki_x100;                       /**< Ki * 100 */
    uint16_t kff_x100;                      /**< Kff * 100 */
} agv_payload_diag_pi_tune_t;

typedef struct {
    uint8_t mode;                           /**< 0: SENSOR, 1: CALIB, 2: TRACK, 3: STOP */
    uint32_t duration_ms;                   /**< Thời gian test */
    int16_t base_rpm_x10;                   /**< Vận tốc nền (RPM * 10) */
} agv_payload_diag_line_test_t;

typedef struct {
    uint8_t acked_msg_id;                   /**< ID của thông điệp được phản hồi */
    uint8_t status_code;                    /**< Mã kết quả: 0=OK, xem agv_ack_status_t */
} agv_payload_ack_t;

typedef struct {
    uint8_t state;                          /**< Trạng thái: xem agv_mission_state_t */
    uint8_t current_step_idx;               /**< Chỉ số bước hiện tại (0..N-1) */
    uint16_t reserved;                      /**< Dự phòng căn chỉnh 4 bytes */
} agv_payload_mission_status_t;

typedef struct {
    int16_t left_rpm_x10;
    int16_t right_rpm_x10;
    uint8_t ir_raw_mask;
    int16_t odom_x_mm;
    int16_t odom_y_mm;
    int16_t odom_theta_deg_x10;
} agv_payload_telemetry_t;

#pragma pack(pop)

/* ========================================================================= */
/* Cấu Trúc Khung Truyền Đã Phân Tích & Bộ Phân Giải (Parser)                */
/* ========================================================================= */
typedef struct {
    uint8_t msg_id;
    uint8_t length;
    uint8_t payload[AGV_PROTOCOL_MAX_PAYLOAD];
} agv_frame_t;

typedef enum {
    AGV_PARSE_NEED_MORE = 0,
    AGV_PARSE_OK,
    AGV_PARSE_ERR_CRC,
    AGV_PARSE_ERR_LENGTH
} agv_parse_status_t;

typedef struct {
    uint8_t state;
    uint8_t msg_id;
    uint8_t length;
    uint8_t payload_idx;
    uint8_t payload[AGV_PROTOCOL_MAX_PAYLOAD];
    uint16_t crc_rx;
} agv_parser_t;

/* ========================================================================= */
/* API Phân Tích & Đóng Gói (Parser & Packing Functions)                     */
/* ========================================================================= */

/**
 * @brief Tính giá trị CRC16-CCITT chuẩn (Poly: 0x1021, Init: 0xFFFF)
 */
uint16_t agv_crc16_ccitt(const uint8_t *data, size_t len, uint16_t init_val);

/**
 * @brief Khởi tạo máy trạng thái parser
 */
void agv_parser_init(agv_parser_t *parser);

/**
 * @brief Xử lý một byte từ luồng UART nhận được
 * @param parser Con trỏ tới bộ parser
 * @param byte Byte nhận được từ UART
 * @param out_frame Con trỏ lưu frame khi phân tích thành công
 * @return agv_parse_status_t Trạng thái: NEED_MORE, OK, ERR_CRC, ERR_LENGTH
 */
agv_parse_status_t agv_parser_feed_byte(agv_parser_t *parser, uint8_t byte, agv_frame_t *out_frame);

/**
 * @brief Đóng gói khung truyền tổng quát với Header, Payload và CRC16
 */
size_t agv_pack_frame(uint8_t msg_id, const void *payload, uint8_t payload_len, uint8_t *out_buf, size_t max_len);

/* Các hàm trợ giúp đóng gói từng loại thông điệp cụ thể */
size_t agv_pack_ping(uint8_t *out_buf, size_t max_len);
size_t agv_pack_pong(uint8_t *out_buf, size_t max_len);
size_t agv_pack_ack(uint8_t acked_msg_id, uint8_t status, uint8_t *out_buf, size_t max_len);
size_t agv_pack_emergency_stop(uint8_t *out_buf, size_t max_len);
size_t agv_pack_route_load(float target_rpm, const uint8_t *actions, uint8_t count, uint8_t *out_buf, size_t max_len);
size_t agv_pack_route_start(uint8_t *out_buf, size_t max_len);
size_t agv_pack_route_cancel(uint8_t *out_buf, size_t max_len);
size_t agv_pack_mission_status(uint8_t state, uint8_t current_step, uint8_t *out_buf, size_t max_len);
size_t agv_pack_log(const char *text, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_step_test(uint8_t motor_id, int16_t pwm, uint32_t duration_ms, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_set_speed(int16_t left_pwm, int16_t right_pwm, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_set_rpm(float left_rpm, float right_rpm, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_get_enc(uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_enc_data(int32_t left_enc, int32_t right_enc, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_reset_enc(uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_pi_tune(float kp, float ki, float kff, uint8_t *out_buf, size_t max_len);
size_t agv_pack_diag_line_test(uint8_t mode, uint32_t duration_ms, float base_rpm, uint8_t *out_buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_PROTOCOL_H */
