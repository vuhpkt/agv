/**
 * @file app_comms.c
 * @brief Bộ điều phối lệnh nhị phân AGV qua UART với ESP32
 */

#include "app_comms.h"
#include "comms_protocol.h"
#include "bsp_uart.h"
#include "bsp_board.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "app_motor_control.h"
#include "app_line_follow.h"
#include "diag_motor.h"
#include "diag_ir.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static agv_parser_t s_parser;
static uint8_t s_tx_frame_buf[AGV_PROTOCOL_MAX_FRAME_SIZE];

static void app_comms_handle_frame(const agv_frame_t *frame) {
    switch (frame->msg_id) {
        case AGV_MSG_PING:
            BSP_LED_Toggle();
            {
                size_t len = agv_pack_pong(s_tx_frame_buf, sizeof(s_tx_frame_buf));
                if (len > 0) BSP_UART_Write(s_tx_frame_buf, len);
            }
            break;

        case AGV_MSG_EMERGENCY_STOP:
            Diag_Motor_Stop();
            Diag_IR_Stop();
            App_LineFollow_CancelRoute();
            App_MotorControl_Stop();
            BSP_Motor_Brake(MOTOR_LEFT);
            BSP_Motor_Brake(MOTOR_RIGHT);
            App_Comms_SendAck(AGV_MSG_EMERGENCY_STOP, AGV_ACK_OK);
            break;

        case AGV_MSG_ROUTE_LOAD:
            if (frame->length >= 3) {
                const agv_payload_route_load_t *route = (const agv_payload_route_load_t*)frame->payload;
                float rpm = (float)route->target_rpm_x10 / 10.0f;
                bool ok = App_LineFollow_LoadRoute(route->actions, route->count, rpm);
                App_Comms_SendAck(AGV_MSG_ROUTE_LOAD, ok ? AGV_ACK_OK : AGV_ACK_ERR_INVALID_ARGS);
            } else {
                App_Comms_SendAck(AGV_MSG_ROUTE_LOAD, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        case AGV_MSG_ROUTE_START:
            App_LineFollow_StartRoute();
            App_Comms_SendAck(AGV_MSG_ROUTE_START, AGV_ACK_OK);
            break;

        case AGV_MSG_ROUTE_CANCEL:
            App_LineFollow_CancelRoute();
            App_Comms_SendAck(AGV_MSG_ROUTE_CANCEL, AGV_ACK_OK);
            break;

        case AGV_MSG_DIAG_STEP_TEST:
            if (frame->length >= sizeof(agv_payload_diag_step_test_t)) {
                const agv_payload_diag_step_test_t *step = (const agv_payload_diag_step_test_t*)frame->payload;
                Diag_Motor_Start((Motor_ID_t)step->motor_id, step->pwm, step->duration_ms);
                App_Comms_SendAck(AGV_MSG_DIAG_STEP_TEST, AGV_ACK_OK);
            } else {
                App_Comms_SendAck(AGV_MSG_DIAG_STEP_TEST, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        case AGV_MSG_DIAG_SET_SPEED:
            if (frame->length >= sizeof(agv_payload_diag_set_speed_t)) {
                const agv_payload_diag_set_speed_t *sp = (const agv_payload_diag_set_speed_t*)frame->payload;
                App_MotorControl_Stop();
                BSP_Motor_SetSpeeds(sp->left_pwm, sp->right_pwm);
                App_Comms_SendAck(AGV_MSG_DIAG_SET_SPEED, AGV_ACK_OK);
            } else {
                App_Comms_SendAck(AGV_MSG_DIAG_SET_SPEED, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        case AGV_MSG_DIAG_SET_RPM:
            if (frame->length >= sizeof(agv_payload_diag_set_rpm_t)) {
                const agv_payload_diag_set_rpm_t *rpm_p = (const agv_payload_diag_set_rpm_t*)frame->payload;
                float l_rpm = (float)rpm_p->left_rpm_x10 / 10.0f;
                float r_rpm = (float)rpm_p->right_rpm_x10 / 10.0f;
                App_MotorControl_SetTargetRPM(l_rpm, r_rpm);
                App_Comms_SendAck(AGV_MSG_DIAG_SET_RPM, AGV_ACK_OK);
            } else {
                App_Comms_SendAck(AGV_MSG_DIAG_SET_RPM, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        case AGV_MSG_DIAG_GET_ENC: {
            int32_t enc_l = BSP_Encoder_GetCount(ENCODER_LEFT);
            int32_t enc_r = BSP_Encoder_GetCount(ENCODER_RIGHT);
            App_Comms_SendEncData(enc_l, enc_r);
            break;
        }

        case AGV_MSG_DIAG_RESET_ENC:
            BSP_Encoder_ResetAll();
            App_Comms_SendAck(AGV_MSG_DIAG_RESET_ENC, AGV_ACK_OK);
            break;

        case AGV_MSG_DIAG_PI_TUNE:
            if (frame->length >= sizeof(agv_payload_diag_pi_tune_t)) {
                const agv_payload_diag_pi_tune_t *tune = (const agv_payload_diag_pi_tune_t*)frame->payload;
                float kp = (float)tune->kp_x100 / 100.0f;
                float ki = (float)tune->ki_x100 / 100.0f;
                float kff = (float)tune->kff_x100 / 100.0f;
                App_MotorControl_SetGains(kp, ki, kff);
                App_Comms_SendAck(AGV_MSG_DIAG_PI_TUNE, AGV_ACK_OK);
            } else {
                App_Comms_SendAck(AGV_MSG_DIAG_PI_TUNE, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        case AGV_MSG_DIAG_LINE_TEST:
            if (frame->length >= sizeof(agv_payload_diag_line_test_t)) {
                const agv_payload_diag_line_test_t *lt = (const agv_payload_diag_line_test_t*)frame->payload;
                if (lt->mode == 3) {
                    Diag_IR_Stop();
                } else {
                    float base_rpm = (float)lt->base_rpm_x10 / 10.0f;
                    Diag_IR_Start((Diag_IR_Mode_t)lt->mode, lt->duration_ms, base_rpm);
                }
                App_Comms_SendAck(AGV_MSG_DIAG_LINE_TEST, AGV_ACK_OK);
            } else {
                App_Comms_SendAck(AGV_MSG_DIAG_LINE_TEST, AGV_ACK_ERR_INVALID_ARGS);
            }
            break;

        default:
            App_Comms_SendAck(frame->msg_id, AGV_ACK_ERR_UNKNOWN_CMD);
            break;
    }
}

void App_Comms_Init(void) {
    agv_parser_init(&s_parser);
}

void App_Comms_Process(void) {
    uint8_t byte;
    while (BSP_UART_ReadByte(&byte)) {
        agv_frame_t frame;
        agv_parse_status_t status = agv_parser_feed_byte(&s_parser, byte, &frame);
        if (status == AGV_PARSE_OK) {
            app_comms_handle_frame(&frame);
        } else if (status == AGV_PARSE_ERR_CRC) {
            App_Comms_SendAck(s_parser.msg_id, AGV_ACK_ERR_CRC);
        }
    }
}

void App_Comms_SendLog(const char *fmt, ...) {
    char text_buf[AGV_PROTOCOL_MAX_PAYLOAD];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text_buf, sizeof(text_buf), fmt, args);
    va_end(args);

    size_t len = agv_pack_log(text_buf, s_tx_frame_buf, sizeof(s_tx_frame_buf));
    if (len > 0) {
        BSP_UART_Write(s_tx_frame_buf, len);
    }
}

void App_Comms_SendAck(uint8_t acked_msg_id, uint8_t status) {
    size_t len = agv_pack_ack(acked_msg_id, status, s_tx_frame_buf, sizeof(s_tx_frame_buf));
    if (len > 0) {
        BSP_UART_Write(s_tx_frame_buf, len);
    }
}

void App_Comms_SendMissionStatus(uint8_t state, uint8_t current_step) {
    size_t len = agv_pack_mission_status(state, current_step, s_tx_frame_buf, sizeof(s_tx_frame_buf));
    if (len > 0) {
        BSP_UART_Write(s_tx_frame_buf, len);
    }
}

void App_Comms_SendEncData(int32_t left, int32_t right) {
    size_t len = agv_pack_diag_enc_data(left, right, s_tx_frame_buf, sizeof(s_tx_frame_buf));
    if (len > 0) {
        BSP_UART_Write(s_tx_frame_buf, len);
    }
}
