#include "line_tracker.h"
#include <stddef.h>

void line_tracker_init(line_tracker_t *tracker, const line_pid_cfg_t *pid_cfg) {
    if (tracker == NULL) {
        return;
    }

    if (pid_cfg != NULL) {
        tracker->pid_cfg = *pid_cfg;
    } else {
        tracker->pid_cfg.kp = 0.25f;
        tracker->pid_cfg.ki = 0.01f;
        tracker->pid_cfg.kd = 0.05f;
        tracker->pid_cfg.out_max = 80.0f;
    }

    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        tracker->calib.min_raw[i] = 100;
        tracker->calib.max_raw[i] = 3500;
        tracker->calib.threshold[i] = 1800;
    }
    tracker->calib.is_white_line = false; /* Mặc định vạch đen trên sàn sáng màu */

    line_tracker_reset(tracker);
}

void line_tracker_set_calib(line_tracker_t *tracker, const line_calib_t *calib) {
    if (tracker == NULL || calib == NULL) {
        return;
    }
    tracker->calib = *calib;
}

void line_tracker_update_calib_minmax(line_tracker_t *tracker, const uint16_t raw[LINE_TRACKER_NUM_SENSORS]) {
    if (tracker == NULL || raw == NULL) {
        return;
    }
    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        if (raw[i] < tracker->calib.min_raw[i]) {
            tracker->calib.min_raw[i] = raw[i];
        }
        if (raw[i] > tracker->calib.max_raw[i]) {
            tracker->calib.max_raw[i] = raw[i];
        }
        tracker->calib.threshold[i] = (tracker->calib.min_raw[i] + tracker->calib.max_raw[i]) / 2;
    }
}

void line_tracker_normalize(const line_tracker_t *tracker, const uint16_t raw[LINE_TRACKER_NUM_SENSORS], uint16_t out_norm[LINE_TRACKER_NUM_SENSORS]) {
    if (tracker == NULL || raw == NULL || out_norm == NULL) {
        return;
    }

    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        uint16_t min_v = tracker->calib.min_raw[i];
        uint16_t max_v = tracker->calib.max_raw[i];
        uint32_t span = (max_v > min_v) ? (uint32_t)(max_v - min_v) : 1;

        uint16_t val = raw[i];
        if (val < min_v) val = min_v;
        if (val > max_v) val = max_v;

        uint32_t norm = ((uint32_t)(val - min_v) * 1000) / span;

        /* Nếu là vạch đen nền sáng (giá trị ADC thấp khi gặp vạch), đảo ngược để vạch đen = 1000 */
        if (!tracker->calib.is_white_line) {
            norm = (norm <= 1000) ? (1000 - norm) : 0;
        }

        if (norm > 1000) norm = 1000;
        out_norm[i] = (uint16_t)norm;
    }
}

int16_t line_tracker_compute_error(line_tracker_t *tracker, const uint16_t norm[LINE_TRACKER_NUM_SENSORS]) {
    if (tracker == NULL || norm == NULL) {
        return 0;
    }

    /* Trọng số vị trí: CH0(R2)=+1000, CH1(R1)=+500, CH2(MID)=0, CH3(L1)=-500, CH4(L2)=-1000 */
    static const int32_t weights[LINE_TRACKER_NUM_SENSORS] = {1000, 500, 0, -500, -1000};
    int32_t sum_weighted = 0;
    int32_t total_intensity = 0;
    int active_count = 0;

    for (int i = 0; i < LINE_TRACKER_NUM_SENSORS; i++) {
        int32_t intensity = (int32_t)norm[i];
        if (intensity > 300) {
            active_count++;
        }
        sum_weighted += weights[i] * intensity;
        total_intensity += intensity;
    }

    /* 1. Nhận diện nút giao / ngã tư: tối thiểu 4 mắt cùng thấy vạch */
    tracker->is_crossline = (active_count >= 4);

    /* 2. Nhận diện mất line: tổng cường độ vạch quá thấp */
    if (total_intensity < 200) {
        tracker->line_lost = true;
        /* Giữ sai số cực đại theo hướng lệch cuối cùng để xe tiếp tục bẻ cua tìm vạch */
        if (tracker->last_valid_error < 0) {
            return -1000;
        } else if (tracker->last_valid_error > 0) {
            return 1000;
        } else {
            return 0;
        }
    }

    tracker->line_lost = false;
    int32_t error = sum_weighted / total_intensity;
    if (error > 1000) error = 1000;
    if (error < -1000) error = -1000;

    tracker->last_valid_error = (int16_t)error;
    return (int16_t)error;
}

float line_tracker_compute_pid(line_tracker_t *tracker, int16_t error, float dt_s) {
    if (tracker == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    float err_f = (float)error;

    /* 1. Proportional term */
    float p_term = tracker->pid_cfg.kp * err_f;

    /* 2. Integral term với Anti-windup */
    tracker->integral += err_f * dt_s;
    float i_limit = tracker->pid_cfg.out_max * 0.5f;
    if (tracker->integral > i_limit) {
        tracker->integral = i_limit;
    } else if (tracker->integral < -i_limit) {
        tracker->integral = -i_limit;
    }
    float i_term = tracker->pid_cfg.ki * tracker->integral;

    /* 3. Derivative term */
    float d_term = tracker->pid_cfg.kd * (err_f - tracker->prev_error) / dt_s;
    tracker->prev_error = err_f;

    /* 4. Tổng điều khiển và bão hòa ngõ ra (Output Clamp) */
    float output = p_term + i_term + d_term;
    if (output > tracker->pid_cfg.out_max) {
        output = tracker->pid_cfg.out_max;
    } else if (output < -tracker->pid_cfg.out_max) {
        output = -tracker->pid_cfg.out_max;
    }

    return output;
}

void line_tracker_reset(line_tracker_t *tracker) {
    if (tracker == NULL) {
        return;
    }
    tracker->integral = 0.0f;
    tracker->prev_error = 0.0f;
    tracker->last_valid_error = 0;
    tracker->line_lost = false;
    tracker->is_crossline = false;
}
