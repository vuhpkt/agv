/**
 * @file app_line_follow.c
 * @brief Điều khiển bám line AGV hỗ trợ chuỗi lộ trình FSM (Route Queue)
 */

#include "app_line_follow.h"
#include "app_line_calib_defaults.h"
#include "bsp_ir.h"
#include "bsp_board.h"
#include "app_motor_control.h"
#include "app_comms.h"
#include "comms_protocol.h"
#include <stddef.h>
#include <string.h>

#define LINE_FOLLOW_LOOP_RATE_MS    10     /**< Chu kỳ điều khiển bám line 10ms (100Hz) */
#define LINE_CALIB_DURATION_MS      3000   /**< Thời gian tự hiệu chuẩn min/max (3 giây) */
#define LINE_LOST_TIMEOUT_MS        2000   /**< Thời gian an toàn ngắt motor khi mất vạch liên tục (2 giây) */
#define CROSSLINE_DEBOUNCE_MS       400    /**< Thời gian debounce sau khi qua ngã rẽ */
#define TURN_MIN_DURATION_MS        350    /**< Thời gian quay tối thiểu để thoát khỏi line cũ */

static line_state_t s_state = LINE_STATE_IDLE;
static line_tracker_t s_tracker;
static float s_base_rpm = 80.0f;
static uint32_t s_last_track_tick = 0;
static uint32_t s_calib_start_tick = 0;
static uint32_t s_lost_start_tick = 0;
static uint16_t s_last_raw[LINE_TRACKER_NUM_SENSORS] = {0};
static uint16_t s_last_norm[LINE_TRACKER_NUM_SENSORS] = {0};
static int16_t s_last_error = 0;

/* Quản lý chuỗi lộ trình (Route Queue) */
static bool s_is_route_mode = false;
static uint8_t s_route_actions[AGV_ROUTE_MAX_ACTIONS] = {0};
static uint8_t s_route_count = 0;
static uint8_t s_route_step = 0;
static uint32_t s_crossline_debounce_until = 0;
static uint32_t s_turn_start_tick = 0;

void App_LineFollow_Init(void) {
    BSP_IR_Init();

    line_pid_cfg_t pid_cfg = {
        .kp = 0.15f,
        .ki = 0.0f,
        .kd = 0.03f,
        .out_max = 60.0f
    };
    line_tracker_init(&s_tracker, &pid_cfg);

    static const uint16_t default_min[LINE_TRACKER_NUM_SENSORS] = APP_LINE_CALIB_MIN_RAW;
    static const uint16_t default_max[LINE_TRACKER_NUM_SENSORS] = APP_LINE_CALIB_MAX_RAW;
    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        s_tracker.calib.min_raw[i] = default_min[i];
        s_tracker.calib.max_raw[i] = default_max[i];
        s_tracker.calib.threshold[i] = (default_min[i] + default_max[i]) / 2;
    }

    s_state = LINE_STATE_IDLE;
    s_is_route_mode = false;
    s_route_count = 0;
    s_route_step = 0;
    s_crossline_debounce_until = 0;
    s_last_track_tick = bsp_time_get_ms();
}

void App_LineFollow_Start(float base_rpm) {
    s_base_rpm = (base_rpm > 0.0f) ? base_rpm : 80.0f;
    s_is_route_mode = false;
    s_tracker.pid_cfg.out_max = (s_base_rpm >= 60.0f) ? s_base_rpm : 60.0f;
    line_tracker_reset(&s_tracker);
    s_lost_start_tick = 0;
    s_crossline_debounce_until = 0;
    s_state = LINE_STATE_TRACKING;
}

void App_LineFollow_Stop(void) {
    s_state = LINE_STATE_IDLE;
    s_lost_start_tick = 0;
    App_MotorControl_Stop();
}

bool App_LineFollow_LoadRoute(const uint8_t *actions, uint8_t count, float target_rpm) {
    if (actions == NULL || count == 0 || count > AGV_ROUTE_MAX_ACTIONS) {
        return false;
    }
    memcpy(s_route_actions, actions, count);
    s_route_count = count;
    s_route_step = 0;
    s_base_rpm = (target_rpm > 0.0f) ? target_rpm : 80.0f;
    s_tracker.pid_cfg.out_max = (s_base_rpm >= 60.0f) ? s_base_rpm : 60.0f;
    return true;
}

void App_LineFollow_StartRoute(void) {
    if (s_route_count == 0) {
        return;
    }
    s_route_step = 0;
    s_is_route_mode = true;
    s_crossline_debounce_until = 0;
    line_tracker_reset(&s_tracker);
    s_lost_start_tick = 0;
    s_state = LINE_STATE_TRACKING;
    App_Comms_SendMissionStatus(AGV_MISSION_RUNNING, s_route_step);
}

void App_LineFollow_CancelRoute(void) {
    s_is_route_mode = false;
    s_route_count = 0;
    s_route_step = 0;
    App_LineFollow_Stop();
    App_Comms_SendMissionStatus(AGV_MISSION_IDLE, 0);
}

uint8_t App_LineFollow_GetCurrentStep(void) {
    return s_route_step;
}

bool App_LineFollow_IsRouteRunning(void) {
    return s_is_route_mode && (s_state != LINE_STATE_IDLE);
}

void App_LineFollow_StartCalibration(void) {
    s_calib_start_tick = bsp_time_get_ms();
    s_state = LINE_STATE_CALIBRATING;
    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        s_tracker.calib.min_raw[i] = 4095;
        s_tracker.calib.max_raw[i] = 0;
    }
}

void App_LineFollow_Process(void) {
    uint32_t now = bsp_time_get_ms();

    if ((uint32_t)(now - s_last_track_tick) < LINE_FOLLOW_LOOP_RATE_MS) {
        return;
    }
    s_last_track_tick = now;

    /* 1. Xử lý chế độ Tự hiệu chuẩn Min/Max */
    if (s_state == LINE_STATE_CALIBRATING) {
        BSP_IR_GetRaw(s_last_raw);
        line_tracker_update_calib_minmax(&s_tracker, s_last_raw);

        if ((uint32_t)(now - s_calib_start_tick) >= LINE_CALIB_DURATION_MS) {
            s_state = LINE_STATE_IDLE;
        }
        return;
    }

    /* 2. Xử lý trạng thái đang quay đầu tại ngã rẽ */
    if (s_state == LINE_STATE_TURNING_LEFT || s_state == LINE_STATE_TURNING_RIGHT) {
        BSP_IR_GetRaw(s_last_raw);
        line_tracker_normalize(&s_tracker, s_last_raw, s_last_norm);
        line_tracker_compute_error(&s_tracker, s_last_norm);

        float turn_speed = s_base_rpm * 0.6f;
        if (turn_speed < 30.0f) turn_speed = 30.0f;

        float target_l, target_r;
        if (s_state == LINE_STATE_TURNING_LEFT) {
            target_l = -turn_speed;
            target_r = turn_speed;
        } else {
            target_l = turn_speed;
            target_r = -turn_speed;
        }

        /* Khi đã quay quá thời gian tối thiểu và mắt IR giữa bắt được vạch nhánh */
        if ((uint32_t)(now - s_turn_start_tick) >= TURN_MIN_DURATION_MS) {
            bool center_on_line = (s_last_norm[2] > s_tracker.calib.threshold[2]);
            if (center_on_line && !s_tracker.is_crossline) {
                s_state = LINE_STATE_TRACKING;
                s_crossline_debounce_until = now + CROSSLINE_DEBOUNCE_MS;
                s_lost_start_tick = 0;
            }
        }

        App_MotorControl_SetTargetRPM(target_l, target_r);
        return;
    }

    /* 3. Xử lý chế độ Bám line thông thường hoặc qua vạch ngang */
    if (s_state == LINE_STATE_TRACKING || s_state == LINE_STATE_CROSS_LINE || s_state == LINE_STATE_LOST) {
        BSP_IR_GetRaw(s_last_raw);
        line_tracker_normalize(&s_tracker, s_last_raw, s_last_norm);
        s_last_error = line_tracker_compute_error(&s_tracker, s_last_norm);

        /* Nhận diện vạch ngang ngã rẽ */
        if (s_tracker.is_crossline) {
            s_lost_start_tick = 0;

            if (now >= s_crossline_debounce_until) {
                if (s_is_route_mode) {
                    uint8_t action = (s_route_step < s_route_count) ? s_route_actions[s_route_step] : AGV_ACTION_STOP;
                    s_route_step++;

                    if (action == AGV_ACTION_FORWARD) {
                        s_state = LINE_STATE_CROSS_LINE;
                        s_crossline_debounce_until = now + CROSSLINE_DEBOUNCE_MS;
                        if (s_route_step >= s_route_count) {
                            /* Đã hoàn thành bước cuối cùng của lộ trình */
                            App_LineFollow_Stop();
                            s_is_route_mode = false;
                            App_Comms_SendMissionStatus(AGV_MISSION_COMPLETED, s_route_step);
                            return;
                        }
                    } else if (action == AGV_ACTION_TURN_LEFT) {
                        s_state = LINE_STATE_TURNING_LEFT;
                        s_turn_start_tick = now;
                        if (s_route_step >= s_route_count) {
                            s_is_route_mode = false;
                            App_LineFollow_Stop();
                            App_Comms_SendMissionStatus(AGV_MISSION_COMPLETED, s_route_step);
                            return;
                        }
                    } else if (action == AGV_ACTION_TURN_RIGHT) {
                        s_state = LINE_STATE_TURNING_RIGHT;
                        s_turn_start_tick = now;
                        if (s_route_step >= s_route_count) {
                            s_is_route_mode = false;
                            App_LineFollow_Stop();
                            App_Comms_SendMissionStatus(AGV_MISSION_COMPLETED, s_route_step);
                            return;
                        }
                    } else {
                        /* ACTION_STOP: Dừng xe hoàn thành lộ trình */
                        App_LineFollow_Stop();
                        s_is_route_mode = false;
                        App_Comms_SendMissionStatus(AGV_MISSION_COMPLETED, s_route_step);
                        return;
                    }
                } else {
                    s_state = LINE_STATE_CROSS_LINE;
                }
            }
        } else if (s_tracker.line_lost) {
            s_state = LINE_STATE_LOST;
            if (s_lost_start_tick == 0) {
                s_lost_start_tick = now;
            } else if ((uint32_t)(now - s_lost_start_tick) >= LINE_LOST_TIMEOUT_MS) {
                /* Mất vạch quá thời gian cho phép: dừng an toàn */
                App_LineFollow_Stop();
                if (s_is_route_mode) {
                    s_is_route_mode = false;
                    App_Comms_SendMissionStatus(AGV_MISSION_ERROR_LOST, s_route_step);
                }
                return;
            }
        } else {
            s_state = LINE_STATE_TRACKING;
            s_lost_start_tick = 0;
        }

        float target_left;
        float target_right;

        if (s_state == LINE_STATE_CROSS_LINE) {
            /* Đi thẳng ổn định qua giao lộ ngã tư, không đánh lái lệch */
            target_left = s_base_rpm;
            target_right = s_base_rpm;
        } else if (s_state == LINE_STATE_LOST) {
            /* Khi mất vạch: dò lại vạch theo hướng lệch cuối cùng ở tốc độ 60%
             * last_valid_error > 0: vạch bên phải -> bẻ phải (trái nhanh, phải chậm)
             * last_valid_error < 0: vạch bên trái -> bẻ trái (trái chậm, phải nhanh)
             */
            float search_rpm = s_base_rpm * 0.6f;
            float delta_rpm = (s_tracker.last_valid_error > 0) ? 20.0f : -20.0f;
            target_left = search_rpm + delta_rpm;
            target_right = search_rpm - delta_rpm;
        } else {
            /* Bám line thông thường qua Line PID (Hồi tiếp âm ổn định):
             * error > 0 (vạch lệch phải): delta_rpm > 0 -> target_left tăng, target_right giảm -> bẻ PHẢI
             * error < 0 (vạch lệch trái): delta_rpm < 0 -> target_left giảm, target_right tăng -> bẻ TRÁI
             */
            float delta_rpm = line_tracker_compute_pid(&s_tracker, s_last_error, 0.01f);
            target_left = s_base_rpm + delta_rpm;
            target_right = s_base_rpm - delta_rpm;
        }

        App_MotorControl_SetTargetRPM(target_left, target_right);
    }
}

line_state_t App_LineFollow_GetState(void) {
    return s_state;
}

bool App_LineFollow_IsCrossline(void) {
    return s_tracker.is_crossline;
}

int16_t App_LineFollow_GetLastError(void) {
    return s_last_error;
}

void App_LineFollow_GetSensors(uint16_t raw[LINE_TRACKER_NUM_SENSORS], uint16_t norm[LINE_TRACKER_NUM_SENSORS], int16_t *error) {
    if (raw != NULL) {
        for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) raw[i] = s_last_raw[i];
    }
    if (norm != NULL) {
        for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) norm[i] = s_last_norm[i];
    }
    if (error != NULL) {
        *error = s_last_error;
    }
}

void App_LineFollow_SetPID(float kp, float ki, float kd) {
    s_tracker.pid_cfg.kp = kp;
    s_tracker.pid_cfg.ki = ki;
    s_tracker.pid_cfg.kd = kd;
}
