#include "pid.h"
#include <stddef.h>

void pid_init(pid_t *pid, const pid_cfg_t *cfg) {
    if (pid == NULL) {
        return;
    }

    if (cfg != NULL) {
        pid->cfg = *cfg;
    } else {
        pid->cfg.kp = 0.0f;
        pid->cfg.ki = 0.0f;
        pid->cfg.kd = 0.0f;
        pid->cfg.out_min = -1000.0f;
        pid->cfg.out_max = 1000.0f;
        pid->cfg.integral_limit = 1000.0f;
    }

    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float pid_calculate(pid_t *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    /* 1. Tính sai số */
    float error = setpoint - measurement;

    /* 2. Thành phần Tỉ lệ (Proportional) */
    float p_out = pid->cfg.kp * error;

    /* 3. Thành phần Tích phân (Integral) kèm Anti-Windup clamp */
    pid->integral += error * dt;
    if (pid->cfg.integral_limit > 0.0f) {
        if (pid->integral > pid->cfg.integral_limit) {
            pid->integral = pid->cfg.integral_limit;
        } else if (pid->integral < -pid->cfg.integral_limit) {
            pid->integral = -pid->cfg.integral_limit;
        }
    }
    float i_out = pid->cfg.ki * pid->integral;

    /* 4. Thành phần Vi phân (Derivative) */
    float derivative = (error - pid->prev_error) / dt;
    float d_out = pid->cfg.kd * derivative;

    /* 5. Tổng ngõ ra */
    float total_out = p_out + i_out + d_out;

    /* 6. Bão hòa đầu ra (Output Clamping) */
    if (total_out > pid->cfg.out_max) {
        total_out = pid->cfg.out_max;
    } else if (total_out < pid->cfg.out_min) {
        total_out = pid->cfg.out_min;
    }

    /* Lưu trạng thái */
    pid->prev_error = error;
    pid->output = total_out;

    return total_out;
}

void pid_reset(pid_t *pid) {
    if (pid == NULL) {
        return;
    }

    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

