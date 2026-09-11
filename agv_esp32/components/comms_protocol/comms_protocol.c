/**
 * @file comms_protocol.c
 * @brief Bộ phân tích và đóng gói giao thức truyền thông nhị phân AGV (Zero-HAL)
 */

#include "comms_protocol.h"
#include <string.h>

enum {
    STATE_SOF1 = 0,
    STATE_SOF2,
    STATE_MSG_ID,
    STATE_LENGTH,
    STATE_PAYLOAD,
    STATE_CRC_H,
    STATE_CRC_L
};

uint16_t agv_crc16_ccitt(const uint8_t *data, size_t len, uint16_t init_val) {
    uint16_t crc = init_val;
    if (data == NULL) return crc;

    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ 0x1021);
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

void agv_parser_init(agv_parser_t *parser) {
    if (parser == NULL) return;
    parser->state = STATE_SOF1;
    parser->msg_id = 0;
    parser->length = 0;
    parser->payload_idx = 0;
    parser->crc_rx = 0;
    memset(parser->payload, 0, sizeof(parser->payload));
}

agv_parse_status_t agv_parser_feed_byte(agv_parser_t *parser, uint8_t byte, agv_frame_t *out_frame) {
    if (parser == NULL) return AGV_PARSE_ERR_LENGTH;

    switch (parser->state) {
        case STATE_SOF1:
            if (byte == AGV_PROTOCOL_SOF1) {
                parser->state = STATE_SOF2;
            }
            break;

        case STATE_SOF2:
            if (byte == AGV_PROTOCOL_SOF2) {
                parser->state = STATE_MSG_ID;
            } else if (byte == AGV_PROTOCOL_SOF1) {
                parser->state = STATE_SOF2;
            } else {
                parser->state = STATE_SOF1;
            }
            break;

        case STATE_MSG_ID:
            parser->msg_id = byte;
            parser->state = STATE_LENGTH;
            break;

        case STATE_LENGTH:
            parser->length = byte;
            parser->payload_idx = 0;
            if (parser->length > AGV_PROTOCOL_MAX_PAYLOAD) {
                parser->state = STATE_SOF1;
                return AGV_PARSE_ERR_LENGTH;
            } else if (parser->length == 0) {
                parser->state = STATE_CRC_H;
            } else {
                parser->state = STATE_PAYLOAD;
            }
            break;

        case STATE_PAYLOAD:
            parser->payload[parser->payload_idx++] = byte;
            if (parser->payload_idx >= parser->length) {
                parser->state = STATE_CRC_H;
            }
            break;

        case STATE_CRC_H:
            parser->crc_rx = ((uint16_t)byte << 8);
            parser->state = STATE_CRC_L;
            break;

        case STATE_CRC_L:
            parser->crc_rx |= (uint16_t)byte;
            parser->state = STATE_SOF1;

            /* Tính toán CRC kiểm tra (bao gồm MSG_ID, LENGTH, PAYLOAD) */
            uint8_t hdr[2];
            hdr[0] = parser->msg_id;
            hdr[1] = parser->length;
            uint16_t crc_calc = agv_crc16_ccitt(hdr, 2, 0xFFFF);
            if (parser->length > 0) {
                crc_calc = agv_crc16_ccitt(parser->payload, parser->length, crc_calc);
            }

            if (crc_calc == parser->crc_rx) {
                if (out_frame != NULL) {
                    out_frame->msg_id = parser->msg_id;
                    out_frame->length = parser->length;
                    if (parser->length > 0) {
                        memcpy(out_frame->payload, parser->payload, parser->length);
                    }
                }
                return AGV_PARSE_OK;
            } else {
                return AGV_PARSE_ERR_CRC;
            }

        default:
            parser->state = STATE_SOF1;
            break;
    }

    return AGV_PARSE_NEED_MORE;
}

size_t agv_pack_frame(uint8_t msg_id, const void *payload, uint8_t payload_len, uint8_t *out_buf, size_t max_len) {
    if (payload_len > AGV_PROTOCOL_MAX_PAYLOAD) return 0;
    size_t total_len = AGV_PROTOCOL_HEADER_SIZE + (size_t)payload_len + AGV_PROTOCOL_CRC_SIZE;
    if (out_buf == NULL || max_len < total_len) return 0;

    out_buf[0] = AGV_PROTOCOL_SOF1;
    out_buf[1] = AGV_PROTOCOL_SOF2;
    out_buf[2] = msg_id;
    out_buf[3] = payload_len;

    if (payload != NULL && payload_len > 0) {
        memcpy(&out_buf[AGV_PROTOCOL_HEADER_SIZE], payload, payload_len);
    }

    /* CRC tính từ msg_id (offset 2) với độ dài 2 + payload_len */
    uint16_t crc = agv_crc16_ccitt(&out_buf[2], 2 + payload_len, 0xFFFF);
    out_buf[AGV_PROTOCOL_HEADER_SIZE + payload_len] = (uint8_t)(crc >> 8);
    out_buf[AGV_PROTOCOL_HEADER_SIZE + payload_len + 1] = (uint8_t)(crc & 0xFF);

    return total_len;
}

size_t agv_pack_ping(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_PING, NULL, 0, out_buf, max_len);
}

size_t agv_pack_pong(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_PONG, NULL, 0, out_buf, max_len);
}

size_t agv_pack_ack(uint8_t acked_msg_id, uint8_t status, uint8_t *out_buf, size_t max_len) {
    agv_payload_ack_t p;
    p.acked_msg_id = acked_msg_id;
    p.status_code = status;
    return agv_pack_frame(AGV_MSG_ACK, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_emergency_stop(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_EMERGENCY_STOP, NULL, 0, out_buf, max_len);
}

size_t agv_pack_route_load(float target_rpm, const uint8_t *actions, uint8_t count, uint8_t *out_buf, size_t max_len) {
    agv_payload_route_load_t p;
    memset(&p, 0, sizeof(p));
    p.count = (count > AGV_ROUTE_MAX_ACTIONS) ? AGV_ROUTE_MAX_ACTIONS : count;
    p.target_rpm_x10 = (int16_t)(target_rpm * 10.0f + 0.5f);
    if (actions != NULL && p.count > 0) {
        memcpy(p.actions, actions, p.count);
    }
    return agv_pack_frame(AGV_MSG_ROUTE_LOAD, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_route_start(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_ROUTE_START, NULL, 0, out_buf, max_len);
}

size_t agv_pack_route_cancel(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_ROUTE_CANCEL, NULL, 0, out_buf, max_len);
}

size_t agv_pack_mission_status(uint8_t state, uint8_t current_step, uint8_t *out_buf, size_t max_len) {
    agv_payload_mission_status_t p;
    p.state = state;
    p.current_step_idx = current_step;
    p.reserved = 0;
    return agv_pack_frame(AGV_MSG_MISSION_STATUS, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_log(const char *text, uint8_t *out_buf, size_t max_len) {
    size_t str_len = (text != NULL) ? strlen(text) : 0;
    if (str_len > 64) str_len = 64;
    return agv_pack_frame(AGV_MSG_LOG, text, (uint8_t)str_len, out_buf, max_len);
}

size_t agv_pack_diag_step_test(uint8_t motor_id, int16_t pwm, uint32_t duration_ms, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_step_test_t p;
    p.motor_id = motor_id;
    p.pwm = pwm;
    p.duration_ms = duration_ms;
    return agv_pack_frame(AGV_MSG_DIAG_STEP_TEST, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_diag_set_speed(int16_t left_pwm, int16_t right_pwm, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_set_speed_t p;
    p.left_pwm = left_pwm;
    p.right_pwm = right_pwm;
    return agv_pack_frame(AGV_MSG_DIAG_SET_SPEED, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_diag_set_rpm(float left_rpm, float right_rpm, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_set_rpm_t p;
    p.left_rpm_x10 = (int16_t)(left_rpm * 10.0f + 0.5f);
    p.right_rpm_x10 = (int16_t)(right_rpm * 10.0f + 0.5f);
    return agv_pack_frame(AGV_MSG_DIAG_SET_RPM, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_diag_get_enc(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_DIAG_GET_ENC, NULL, 0, out_buf, max_len);
}

size_t agv_pack_diag_enc_data(int32_t left_enc, int32_t right_enc, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_enc_data_t p;
    p.enc_left = left_enc;
    p.enc_right = right_enc;
    return agv_pack_frame(AGV_MSG_DIAG_ENC_DATA, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_diag_reset_enc(uint8_t *out_buf, size_t max_len) {
    return agv_pack_frame(AGV_MSG_DIAG_RESET_ENC, NULL, 0, out_buf, max_len);
}

size_t agv_pack_diag_pi_tune(float kp, float ki, float kff, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_pi_tune_t p;
    p.kp_x100 = (uint16_t)(kp * 100.0f + 0.5f);
    p.ki_x100 = (uint16_t)(ki * 100.0f + 0.5f);
    p.kff_x100 = (uint16_t)(kff * 100.0f + 0.5f);
    return agv_pack_frame(AGV_MSG_DIAG_PI_TUNE, &p, sizeof(p), out_buf, max_len);
}

size_t agv_pack_diag_line_test(uint8_t mode, uint32_t duration_ms, float base_rpm, uint8_t *out_buf, size_t max_len) {
    agv_payload_diag_line_test_t p;
    p.mode = mode;
    p.duration_ms = duration_ms;
    p.base_rpm_x10 = (int16_t)(base_rpm * 10.0f + 0.5f);
    return agv_pack_frame(AGV_MSG_DIAG_LINE_TEST, &p, sizeof(p), out_buf, max_len);
}
