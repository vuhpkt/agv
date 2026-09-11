#include "ble_cmd_dispatcher.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_service.h"
#include "ble_nus.h"

static const char *TAG = "BLE_DISPATCHER";

void ble_cmd_dispatcher_init(void) {
    ESP_LOGI(TAG, "BLE Command Dispatcher initialized");
}

void ble_cmd_dispatcher_handle_text(const uint8_t *data, size_t len, void *user_data) {
    (void)user_data;
    if (data == NULL || len == 0) return;

    char line[256];
    size_t copy_len = (len < sizeof(line) - 1) ? len : (sizeof(line) - 1);
    memcpy(line, data, copy_len);
    line[copy_len] = '\0';

    /* Trim trailing whitespace and CRLF */
    int p_end = (int)strlen(line) - 1;
    while (p_end >= 0 && (line[p_end] == '\r' || line[p_end] == '\n' || line[p_end] == ' ' || line[p_end] == '\t')) {
        line[p_end--] = '\0';
    }

    /* Trim leading whitespace */
    char *p_start = line;
    while (*p_start == ' ' || *p_start == '\t') {
        p_start++;
    }
    if (*p_start == '\0') return;

    ESP_LOGI(TAG, "[BLE CMD RX] %s", p_start);

    char *saveptr = NULL;
    char *cmd = strtok_r(p_start, " \t", &saveptr);
    if (cmd == NULL) return;

    char *args = strtok_r(NULL, "", &saveptr);
    if (args != NULL) {
        while (*args == ' ' || *args == '\t') args++;
        if (*args == '\0') args = NULL;
    }

    if (strcasecmp(cmd, "PING") == 0) {
        uart_service_send_msg(AGV_MSG_PING, NULL, 0);
    } else if (strcasecmp(cmd, "STOP") == 0 || strcasecmp(cmd, "EMG") == 0) {
        uart_service_send_msg(AGV_MSG_EMERGENCY_STOP, NULL, 0);
        ble_nus_send_str("[RESP] EMERGENCY STOP SENT\r\n");
    } else if (strcasecmp(cmd, "STEP_TEST") == 0) {
        char m_str[16] = {0};
        int pwm = 0;
        unsigned int ms = 0;
        if (args == NULL || sscanf(args, "%15s %d %u", m_str, &pwm, &ms) < 3) {
            ble_nus_send_str("[ERR] Usage: STEP_TEST <L|R> <pwm: -1000..1000> <ms>\r\n");
            return;
        }
        uint8_t motor_id = (m_str[0] == 'R' || m_str[0] == 'r' || m_str[0] == '1') ? 1 : 0;
        if (pwm < -1000) pwm = -1000;
        if (pwm > 1000) pwm = 1000;
        agv_payload_diag_step_test_t p;
        p.motor_id = motor_id;
        p.pwm = (int16_t)pwm;
        p.duration_ms = (uint32_t)ms;
        uart_service_send_msg(AGV_MSG_DIAG_STEP_TEST, &p, sizeof(p));
        ble_nus_send_str("[RESP] STEP TEST STARTED\r\n");
    } else if (strcasecmp(cmd, "SET_SPEED") == 0) {
        int l_pwm = 0, r_pwm = 0;
        if (args == NULL || sscanf(args, "%d %d", &l_pwm, &r_pwm) < 2) {
            ble_nus_send_str("[ERR] Usage: SET_SPEED <left: -1000..1000> <right: -1000..1000>\r\n");
            return;
        }
        if (l_pwm < -1000) l_pwm = -1000;
        if (l_pwm > 1000) l_pwm = 1000;
        if (r_pwm < -1000) r_pwm = -1000;
        if (r_pwm > 1000) r_pwm = 1000;
        agv_payload_diag_set_speed_t p;
        p.left_pwm = (int16_t)l_pwm;
        p.right_pwm = (int16_t)r_pwm;
        uart_service_send_msg(AGV_MSG_DIAG_SET_SPEED, &p, sizeof(p));
        ble_nus_send_str("[RESP] SPEED SET\r\n");
    } else if (strcasecmp(cmd, "SET_RPM") == 0) {
        float l_rpm = 0.0f, r_rpm = 0.0f;
        if (args == NULL || sscanf(args, "%f %f", &l_rpm, &r_rpm) < 2) {
            ble_nus_send_str("[ERR] Usage: SET_RPM <left_rpm> <right_rpm>\r\n");
            return;
        }
        agv_payload_diag_set_rpm_t p;
        p.left_rpm_x10 = (int16_t)(l_rpm * 10.0f + (l_rpm >= 0 ? 0.5f : -0.5f));
        p.right_rpm_x10 = (int16_t)(r_rpm * 10.0f + (r_rpm >= 0 ? 0.5f : -0.5f));
        uart_service_send_msg(AGV_MSG_DIAG_SET_RPM, &p, sizeof(p));
        ble_nus_send_str("[RESP] RPM SET\r\n");
    } else if (strcasecmp(cmd, "GET_ENC") == 0) {
        uart_service_send_msg(AGV_MSG_DIAG_GET_ENC, NULL, 0);
    } else if (strcasecmp(cmd, "RESET_ENC") == 0) {
        uart_service_send_msg(AGV_MSG_DIAG_RESET_ENC, NULL, 0);
        ble_nus_send_str("[RESP] ENCODER RESET OK\r\n");
    } else if (strcasecmp(cmd, "PI_TUNE") == 0) {
        float kp = 0.0f, ki = 0.0f, kff = 0.0f;
        if (args == NULL || sscanf(args, "%f %f %f", &kp, &ki, &kff) < 3) {
            ble_nus_send_str("[ERR] Usage: PI_TUNE <kp> <ki> <kff>\r\n");
            return;
        }
        agv_payload_diag_pi_tune_t p;
        p.kp_x100 = (uint16_t)(kp * 100.0f + 0.5f);
        p.ki_x100 = (uint16_t)(ki * 100.0f + 0.5f);
        p.kff_x100 = (uint16_t)(kff * 100.0f + 0.5f);
        uart_service_send_msg(AGV_MSG_DIAG_PI_TUNE, &p, sizeof(p));
        ble_nus_send_str("[RESP] PI PARAMS UPDATED\r\n");
    } else if (strcasecmp(cmd, "MONITOR_IR") == 0) {
        unsigned int ms = 15000;
        if (args != NULL) {
            sscanf(args, "%u", &ms);
            if (ms == 0) ms = 15000;
        }
        agv_payload_diag_line_test_t p;
        p.mode = 0; // SENSOR mode
        p.duration_ms = (uint32_t)ms;
        p.base_rpm_x10 = 0;
        uart_service_send_msg(AGV_MSG_DIAG_LINE_TEST, &p, sizeof(p));
        ble_nus_send_str("[RESP] IR MONITOR STARTED\r\n");
    } else if (strcasecmp(cmd, "LINE_CALIB") == 0) {
        unsigned int ms = 10000;
        if (args != NULL) {
            sscanf(args, "%u", &ms);
            if (ms == 0) ms = 10000;
        }
        agv_payload_diag_line_test_t p;
        p.mode = 1; // CALIB mode
        p.duration_ms = (uint32_t)ms;
        p.base_rpm_x10 = 0;
        uart_service_send_msg(AGV_MSG_DIAG_LINE_TEST, &p, sizeof(p));
        ble_nus_send_str("[RESP] LINE CALIBRATION STARTED\r\n");
    } else if (strcasecmp(cmd, "LINE_TEST") == 0) {
        int mode = 0;
        unsigned int ms = 5000;
        float rpm = 0.0f;
        if (args == NULL || sscanf(args, "%d %u %f", &mode, &ms, &rpm) < 3) {
            ble_nus_send_str("[ERR] Usage: LINE_TEST <mode: 0..3> <ms> <rpm>\r\n");
            return;
        }
        agv_payload_diag_line_test_t p;
        p.mode = (uint8_t)mode;
        p.duration_ms = (uint32_t)ms;
        p.base_rpm_x10 = (int16_t)(rpm * 10.0f + (rpm >= 0 ? 0.5f : -0.5f));
        uart_service_send_msg(AGV_MSG_DIAG_LINE_TEST, &p, sizeof(p));
        ble_nus_send_str("[RESP] LINE TEST STARTED\r\n");
    } else if (strcasecmp(cmd, "ROUTE_LOAD") == 0) {
        float rpm = 0.0f;
        char act_str[128] = {0};
        if (args == NULL || sscanf(args, "%f %127s", &rpm, act_str) < 2) {
            ble_nus_send_str("[ERR] Usage: ROUTE_LOAD <rpm> <actions: F,R,L,S,...>\r\n");
            return;
        }
        agv_payload_route_load_t p;
        memset(&p, 0, sizeof(p));
        p.target_rpm_x10 = (int16_t)(rpm * 10.0f + 0.5f);
        uint8_t count = 0;
        for (size_t i = 0; act_str[i] != '\0' && count < AGV_ROUTE_MAX_ACTIONS; i++) {
            char c = act_str[i];
            if (c == 'F' || c == 'f') p.actions[count++] = AGV_ACTION_FORWARD;
            else if (c == 'L' || c == 'l') p.actions[count++] = AGV_ACTION_TURN_LEFT;
            else if (c == 'R' || c == 'r') p.actions[count++] = AGV_ACTION_TURN_RIGHT;
            else if (c == 'S' || c == 's') p.actions[count++] = AGV_ACTION_STOP;
            else if (c == ',' || c == ' ') continue;
        }
        p.count = count;
        if (count == 0) {
            ble_nus_send_str("[ERR] No valid actions found (allowed: F, R, L, S)\r\n");
            return;
        }
        uart_service_send_msg(AGV_MSG_ROUTE_LOAD, &p, sizeof(p));
        ble_nus_send_str("[RESP] ROUTE LOADED\r\n");
    } else if (strcasecmp(cmd, "ROUTE_START") == 0) {
        uart_service_send_msg(AGV_MSG_ROUTE_START, NULL, 0);
        ble_nus_send_str("[RESP] ROUTE STARTED\r\n");
    } else if (strcasecmp(cmd, "ROUTE_CANCEL") == 0) {
        uart_service_send_msg(AGV_MSG_ROUTE_CANCEL, NULL, 0);
        ble_nus_send_str("[RESP] ROUTE CANCELLED\r\n");
    } else if (strcasecmp(cmd, "ROUTE_TEST") == 0) {
        agv_payload_route_load_t p;
        memset(&p, 0, sizeof(p));
        p.count = 4;
        p.target_rpm_x10 = 600; // 60 RPM
        p.actions[0] = AGV_ACTION_FORWARD;
        p.actions[1] = AGV_ACTION_TURN_RIGHT;
        p.actions[2] = AGV_ACTION_FORWARD;
        p.actions[3] = AGV_ACTION_STOP;
        uart_service_send_msg(AGV_MSG_ROUTE_LOAD, &p, sizeof(p));
        vTaskDelay(pdMS_TO_TICKS(10));
        uart_service_send_msg(AGV_MSG_ROUTE_START, NULL, 0);
        ble_nus_send_str("[RESP] ROUTE TEST STARTED (F,R,F,S @ 60 RPM)\r\n");
    } else if (strcasecmp(cmd, "HELP") == 0) {
        const char *help_msg =
            "[HELP] AGV Commands:\r\n"
            "  PING\r\n"
            "  STOP / EMG\r\n"
            "  STEP_TEST <L|R> <pwm> <ms>\r\n"
            "  SET_SPEED <left> <right>\r\n"
            "  SET_RPM <left_rpm> <right_rpm>\r\n"
            "  GET_ENC\r\n"
            "  RESET_ENC\r\n"
            "  PI_TUNE <kp> <ki> <kff>\r\n"
            "  MONITOR_IR [ms]\r\n"
            "  LINE_CALIB [ms]\r\n"
            "  LINE_TEST <mode> <ms> <rpm>\r\n"
            "  ROUTE_LOAD <rpm> <actions: F,R,L,S>\r\n"
            "  ROUTE_START\r\n"
            "  ROUTE_CANCEL\r\n"
            "  ROUTE_TEST\r\n";
        ble_nus_send_str(help_msg);
    } else {
        char err_msg[128];
        snprintf(err_msg, sizeof(err_msg), "[ERR] Unknown command: %s. Type HELP for commands\r\n", cmd);
        ble_nus_send_str(err_msg);
    }
}

void ble_cmd_dispatcher_handle_frame(const agv_frame_t *frame, void *user_data) {
    (void)user_data;
    if (frame == NULL) return;

    char resp[256];

    switch (frame->msg_id) {
        case AGV_MSG_PONG:
            ble_nus_send_str("[RESP] PONG (STM32 Alive)\r\n");
            break;

        case AGV_MSG_ACK:
            if (frame->length >= sizeof(agv_payload_ack_t)) {
                const agv_payload_ack_t *ack = (const agv_payload_ack_t *)frame->payload;
                const char *st_str = "UNKNOWN";
                switch (ack->status_code) {
                    case AGV_ACK_OK: st_str = "OK"; break;
                    case AGV_ACK_ERR_CRC: st_str = "ERR_CRC"; break;
                    case AGV_ACK_ERR_UNKNOWN_CMD: st_str = "ERR_UNKNOWN_CMD"; break;
                    case AGV_ACK_ERR_INVALID_ARGS: st_str = "ERR_INVALID_ARGS"; break;
                    case AGV_ACK_ERR_BUSY: st_str = "ERR_BUSY"; break;
                    default: break;
                }
                snprintf(resp, sizeof(resp), "[ACK] MsgID: 0x%02X, Status: %s (Code: %d)\r\n",
                         ack->acked_msg_id, st_str, ack->status_code);
                ble_nus_send_str(resp);
            }
            break;

        case AGV_MSG_LOG: {
            char log_text[130];
            uint8_t len = (frame->length < 128) ? frame->length : 128;
            memcpy(log_text, frame->payload, len);
            log_text[len] = '\0';

            /* If telemetry prefix DATA: pass through directly */
            if (strncmp(log_text, "DATA:", 5) == 0) {
                int l = (int)strlen(log_text);
                while (l > 0 && (log_text[l-1] == '\r' || log_text[l-1] == '\n')) {
                    log_text[--l] = '\0';
                }
                snprintf(resp, sizeof(resp), "%s\r\n", log_text);
                ble_nus_send_str(resp);
            } else {
                snprintf(resp, sizeof(resp), "[STM32 LOG] %s\r\n", log_text);
                ble_nus_send_str(resp);
            }
            break;
        }

        case AGV_MSG_MISSION_STATUS:
            if (frame->length >= sizeof(agv_payload_mission_status_t)) {
                const agv_payload_mission_status_t *ms = (const agv_payload_mission_status_t *)frame->payload;
                const char *st_str = "UNKNOWN";
                switch (ms->state) {
                    case AGV_MISSION_IDLE: st_str = "IDLE"; break;
                    case AGV_MISSION_RUNNING: st_str = "RUNNING"; break;
                    case AGV_MISSION_COMPLETED: st_str = "COMPLETED"; break;
                    case AGV_MISSION_ERROR_LOST: st_str = "ERROR_LOST"; break;
                    case AGV_MISSION_ERROR_TIMEOUT: st_str = "ERROR_TIMEOUT"; break;
                    default: break;
                }
                snprintf(resp, sizeof(resp), "[MISSION] State: %s, Step: %d\r\n", st_str, ms->current_step_idx);
                ble_nus_send_str(resp);
            }
            break;

        case AGV_MSG_DIAG_ENC_DATA:
            if (frame->length >= sizeof(agv_payload_diag_enc_data_t)) {
                const agv_payload_diag_enc_data_t *enc = (const agv_payload_diag_enc_data_t *)frame->payload;
                snprintf(resp, sizeof(resp), "[ENCODER] Left: %ld | Right: %ld\r\n",
                         (long)enc->enc_left, (long)enc->enc_right);
                ble_nus_send_str(resp);
            }
            break;

        case AGV_MSG_TELEMETRY:
            if (frame->length >= sizeof(agv_payload_telemetry_t)) {
                const agv_payload_telemetry_t *t = (const agv_payload_telemetry_t *)frame->payload;
                snprintf(resp, sizeof(resp),
                         "[TELEMETRY] L_RPM: %.1f, R_RPM: %.1f, IR: 0x%02X, X: %d, Y: %d, Theta: %.1f\r\n",
                         t->left_rpm_x10 / 10.0f, t->right_rpm_x10 / 10.0f,
                         t->ir_raw_mask, t->odom_x_mm, t->odom_y_mm,
                         t->odom_theta_deg_x10 / 10.0f);
                ble_nus_send_str(resp);
            }
            break;

        default:
            snprintf(resp, sizeof(resp), "[FRAME] MsgID: 0x%02X, Len: %u\r\n", frame->msg_id, frame->length);
            ble_nus_send_str(resp);
            break;
    }
}
