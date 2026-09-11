/**
 * @file diag_ir.c
 * @brief Triển khai module kiểm tra độc lập chức năng dò line của mảng cảm biến 5 mắt IR
 */

#include "diag_ir.h"
#include "bsp_ir.h"
#include "bsp_board.h"
#include "bsp_motor.h"
#include "line_tracker.h"
#include "app_motor_control.h"
#include "app_line_calib_defaults.h"
#include "app_comms.h"
#include <stdio.h>
#include <string.h>

static bool s_running = false;
static Diag_IR_Mode_t s_mode = DIAG_IR_MODE_SENSOR;
static uint32_t s_duration_ms = DIAG_IR_DEFAULT_DURATION_MS;
static float s_base_rpm = DIAG_IR_DEFAULT_BASE_RPM;
static uint32_t s_start_tick = 0;
static uint32_t s_last_sample_tick = 0;

static line_tracker_t s_tracker;
static Diag_IR_Result_t s_result;

static const char* get_mode_name(Diag_IR_Mode_t mode) {
    switch (mode) {
        case DIAG_IR_MODE_SENSOR: return "SENSOR_MONITOR";
        case DIAG_IR_MODE_CALIB:  return "OPTICAL_CALIB";
        case DIAG_IR_MODE_TRACK:  return "TRACK_RUN";
        default:                  return "UNKNOWN";
    }
}

void Diag_IR_Init(void) {
    BSP_IR_Init();
    s_running = false;
    s_mode = DIAG_IR_MODE_SENSOR;
    s_duration_ms = DIAG_IR_DEFAULT_DURATION_MS;
    s_base_rpm = DIAG_IR_DEFAULT_BASE_RPM;
    s_start_tick = 0;
    s_last_sample_tick = 0;

    line_pid_cfg_t default_pid = {
        .kp = 0.08f,
        .ki = 0.005f,
        .kd = 0.03f,
        .out_max = 40.0f
    };
    line_tracker_init(&s_tracker, &default_pid);

    static const uint16_t default_min[LINE_TRACKER_NUM_SENSORS] = APP_LINE_CALIB_MIN_RAW;
    static const uint16_t default_max[LINE_TRACKER_NUM_SENSORS] = APP_LINE_CALIB_MAX_RAW;
    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        s_tracker.calib.min_raw[i] = default_min[i];
        s_tracker.calib.max_raw[i] = default_max[i];
        s_tracker.calib.threshold[i] = (default_min[i] + default_max[i]) / 2;
    }

    memset(&s_result, 0, sizeof(s_result));
    s_result.status = DIAG_IR_STATE_IDLE;
}

void Diag_IR_Start(Diag_IR_Mode_t mode, uint32_t duration_ms, float base_rpm) {
    s_mode = mode;
    s_duration_ms = (duration_ms > 0) ? duration_ms : DIAG_IR_DEFAULT_DURATION_MS;
    s_base_rpm = (base_rpm > 0.0f) ? base_rpm : DIAG_IR_DEFAULT_BASE_RPM;

    s_start_tick = bsp_time_get_ms();
    s_last_sample_tick = s_start_tick;

    memset(&s_result, 0, sizeof(s_result));
    s_result.status = DIAG_IR_STATE_RUNNING;
    s_result.mode = mode;
    s_result.duration_ms = s_duration_ms;

    for (int i = 0; i < BSP_IR_NUM_SENSORS; i++) {
        s_result.min_raw[i] = 4095;
        s_result.max_raw[i] = 0;
    }

    s_running = true;
    AGV_LOG("[DIAG_IR_START] %s dur=%u ms rpm=%.1f", get_mode_name(mode), (unsigned)s_duration_ms, s_base_rpm);
}

void Diag_IR_StartDefault(void) {
    Diag_IR_Start(DIAG_IR_MODE_SENSOR, DIAG_IR_DEFAULT_DURATION_MS, DIAG_IR_DEFAULT_BASE_RPM);
}

void Diag_IR_Stop(void) {
    if (!s_running) {
        return;
    }

    s_running = false;
    s_result.status = DIAG_IR_STATE_IDLE;

    if (s_mode == DIAG_IR_MODE_TRACK) {
        App_MotorControl_Stop();
        BSP_Motor_Stop(MOTOR_LEFT);
        BSP_Motor_Stop(MOTOR_RIGHT);
    }

    AGV_LOG("[DIAG_IR_STOP] Aborted");
}

bool Diag_IR_IsRunning(void) {
    return s_running;
}

const Diag_IR_Result_t* Diag_IR_GetLastResult(void) {
    return &s_result;
}

void Diag_IR_Process(void) {
    if (!s_running) {
        return;
    }

    uint32_t now = bsp_time_get_ms();
    if ((uint32_t)(now - s_last_sample_tick) < DIAG_IR_SAMPLE_RATE_MS) {
        return;
    }
    s_last_sample_tick = now;

    uint16_t raw[BSP_IR_NUM_SENSORS];
    BSP_IR_GetRaw(raw);

    for (int i = 0; i < BSP_IR_NUM_SENSORS; i++) {
        if (raw[i] < s_result.min_raw[i]) s_result.min_raw[i] = raw[i];
        if (raw[i] > s_result.max_raw[i]) s_result.max_raw[i] = raw[i];
    }

    if (s_mode == DIAG_IR_MODE_CALIB) {
        line_tracker_update_calib_minmax(&s_tracker, raw);
    }

    uint16_t norm[BSP_IR_NUM_SENSORS];
    line_tracker_normalize(&s_tracker, raw, norm);
    int16_t err = line_tracker_compute_error(&s_tracker, norm);
    s_result.last_error = err;

    if (!s_tracker.line_lost) s_result.line_detected = true;
    if (s_tracker.is_crossline) s_result.crossline_detected = true;

    if (s_mode == DIAG_IR_MODE_TRACK) {
        float delta_v = line_tracker_compute_pid(&s_tracker, err, 0.01f);
        float v_l = s_base_rpm - delta_v;
        float v_r = s_base_rpm + delta_v;
        App_MotorControl_SetTargetRPM(v_l, v_r);
    }

    uint32_t elapsed_ms = now - s_start_tick;
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "DATA:LINE,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u\r\n",
                       (unsigned long)elapsed_ms,
                       raw[0], raw[1], raw[2], raw[3], raw[4],
                       norm[0], norm[1], norm[2], norm[3], norm[4],
                       err, (unsigned int)(!s_tracker.line_lost));
    if (len > 0) {
        AGV_LOG("%s", buf);
    }

    s_result.total_samples++;

    if (elapsed_ms >= s_duration_ms) {
        s_running = false;

        if (s_mode == DIAG_IR_MODE_TRACK) {
            App_MotorControl_Stop();
            BSP_Motor_Stop(MOTOR_LEFT);
            BSP_Motor_Stop(MOTOR_RIGHT);
        }

        bool sensors_ok = true;
        for (int i = 0; i < BSP_IR_NUM_SENSORS; i++) {
            if (s_result.max_raw[i] <= s_result.min_raw[i] + DIAG_IR_MIN_DYNAMIC_RANGE) {
                sensors_ok = false;
            }
        }
        s_result.all_sensors_responding = sensors_ok;
        s_result.status = sensors_ok ? DIAG_IR_STATE_PASSED : DIAG_IR_STATE_FAILED;

        AGV_LOG("[DIAG_IR_END] Status: %s | Samples: %u | Line: %s",
                sensors_ok ? "PASS" : "FAIL",
                (unsigned int)s_result.total_samples,
                s_result.line_detected ? "YES" : "NO");
    }
}
