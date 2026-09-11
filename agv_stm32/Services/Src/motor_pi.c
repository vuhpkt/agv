/**
 * @file motor_pi.c
 * @brief Triển khai Component Bộ điều khiển vận tốc Motor PI cho AGV
 */

#include "motor_pi.h"
#include <math.h>
#include <stddef.h>

void motor_pi_get_default_cfg(motor_pi_cfg_t *cfg) {
    if (cfg == NULL) {
        return;
    }

    cfg->kp = MOTOR_PI_DEFAULT_KP;
    cfg->ki = MOTOR_PI_DEFAULT_KI;
    cfg->k_ff = MOTOR_PI_DEFAULT_KFF;
    cfg->out_min = MOTOR_PI_DEFAULT_OUT_MIN;
    cfg->out_max = MOTOR_PI_DEFAULT_OUT_MAX;
    cfg->integral_limit = MOTOR_PI_DEFAULT_I_MAX;
    cfg->deadzone_pwm = MOTOR_PI_DEFAULT_DEADZONE;
    cfg->max_accel_rpm_s = MOTOR_PI_DEFAULT_ACCEL_LIM;
}

void motor_pi_tune_from_plant(motor_pi_cfg_t *cfg, float k, float tau_s, float desired_ts_s) {
    if (cfg == NULL) {
        return;
    }

    /* Đảm bảo tham số nhận dạng hợp lệ */
    float k_abs = fabsf(k);
    if (k_abs < 0.0001f) k_abs = 0.2699f;
    if (tau_s < 0.001f) tau_s = 0.0449f;
    if (desired_ts_s < 0.020f) desired_ts_s = 0.100f;

    /*
     * Phương pháp gán cực (Pole-Placement):
     * Chọn zero của PI triệt tiêu pole của động cơ: Ki / Kp = 1 / tau => Ti = tau
     * Hàm truyền hệ kín bậc 1: Tcl(s) = 1 / (tau_cl * s + 1)
     * với tau_cl = desired_ts / 3.0 (tiêu chuẩn 95% đáp ứng trong desired_ts)
     */
    float tau_cl = desired_ts_s / 3.0f;
    float kp = tau_s / (k_abs * tau_cl);
    float ki = kp / tau_s;
    float k_ff = 1.0f / k_abs;

    cfg->kp = kp;
    cfg->ki = ki;
    cfg->k_ff = k_ff;
    cfg->out_min = MOTOR_PI_DEFAULT_OUT_MIN;
    cfg->out_max = MOTOR_PI_DEFAULT_OUT_MAX;
    cfg->integral_limit = MOTOR_PI_DEFAULT_I_MAX;
    cfg->deadzone_pwm = MOTOR_PI_DEFAULT_DEADZONE;
    cfg->max_accel_rpm_s = MOTOR_PI_DEFAULT_ACCEL_LIM;
}

void motor_pi_init(motor_pi_t *ctrl, const motor_pi_cfg_t *cfg) {
    if (ctrl == NULL) {
        return;
    }

    if (cfg != NULL) {
        ctrl->cfg = *cfg;
    } else {
        motor_pi_get_default_cfg(&ctrl->cfg);
    }

    motor_pi_reset(ctrl);
}

void motor_pi_reset(motor_pi_t *ctrl) {
    if (ctrl == NULL) {
        return;
    }

    ctrl->target_rpm = 0.0f;
    ctrl->measured_rpm = 0.0f;
    ctrl->error = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->integral = 0.0f;
    ctrl->p_term = 0.0f;
    ctrl->i_term = 0.0f;
    ctrl->ff_term = 0.0f;
    ctrl->output_pwm = 0.0f;
    ctrl->is_saturated = false;
}

void motor_pi_set_gains(motor_pi_t *ctrl, float kp, float ki, float k_ff) {
    if (ctrl == NULL) {
        return;
    }

    ctrl->cfg.kp = kp;
    ctrl->cfg.ki = ki;
    ctrl->cfg.k_ff = k_ff;
}

int16_t motor_pi_update(motor_pi_t *ctrl, float setpoint_rpm, float feedback_rpm, float dt_s) {
    if (ctrl == NULL || dt_s <= 0.0f) {
        return 0;
    }

    ctrl->measured_rpm = feedback_rpm;

    /* 1. Giới hạn dốc gia tốc (Slew Rate Limiter trên tín hiệu đặt) */
    if (ctrl->cfg.max_accel_rpm_s > 0.0f) {
        float max_delta = ctrl->cfg.max_accel_rpm_s * dt_s;
        float diff = setpoint_rpm - ctrl->target_rpm;
        if (diff > max_delta) {
            ctrl->target_rpm += max_delta;
        } else if (diff < -max_delta) {
            ctrl->target_rpm -= max_delta;
        } else {
            ctrl->target_rpm = setpoint_rpm;
        }
    } else {
        ctrl->target_rpm = setpoint_rpm;
    }

    /* Trường hợp đặt dừng hoàn toàn: reset tích phân khi tốc độ đã về 0 */
    if (fabsf(setpoint_rpm) < 0.01f && fabsf(ctrl->target_rpm) < 0.1f) {
        ctrl->target_rpm = 0.0f;
        if (fabsf(feedback_rpm) < 1.0f) {
            ctrl->integral = 0.0f;
            ctrl->output_pwm = 0.0f;
            ctrl->p_term = 0.0f;
            ctrl->i_term = 0.0f;
            ctrl->ff_term = 0.0f;
            ctrl->error = 0.0f;
            ctrl->prev_error = 0.0f;
            ctrl->is_saturated = false;
            return 0;
        }
    }

    /* 2. Tính sai số vận tốc */
    float error = ctrl->target_rpm - feedback_rpm;
    ctrl->error = error;

    /* 3. Thành phần tỉ lệ (Proportional) */
    ctrl->p_term = ctrl->cfg.kp * error;

    /* 4. Thành phần Feedforward (Ưu thế đáp ứng nhanh không trễ) */
    ctrl->ff_term = ctrl->cfg.k_ff * ctrl->target_rpm;

    /*
     * 5. Thành phần tích phân với Anti-Windup Clamping:
     * Chỉ tích lũy nếu hệ thống KHÔNG đang bị bão hòa cùng chiều với sai số
     */
    bool saturated_pos = (ctrl->is_saturated && ctrl->output_pwm >= ctrl->cfg.out_max && error > 0.0f);
    bool saturated_neg = (ctrl->is_saturated && ctrl->output_pwm <= ctrl->cfg.out_min && error < 0.0f);

    if (!saturated_pos && !saturated_neg) {
        ctrl->integral += error * dt_s;

        /* Giới hạn trần tích phân */
        if (ctrl->cfg.integral_limit > 0.0f) {
            if (ctrl->integral > ctrl->cfg.integral_limit) {
                ctrl->integral = ctrl->cfg.integral_limit;
            } else if (ctrl->integral < -ctrl->cfg.integral_limit) {
                ctrl->integral = -ctrl->cfg.integral_limit;
            }
        }
    }
    ctrl->i_term = ctrl->cfg.ki * ctrl->integral;

    /* 6. Bù vùng chết ma sát tĩnh (Deadzone Friction Compensation) */
    float deadzone = 0.0f;
    if (ctrl->target_rpm > 0.5f) {
        deadzone = ctrl->cfg.deadzone_pwm;
    } else if (ctrl->target_rpm < -0.5f) {
        deadzone = -ctrl->cfg.deadzone_pwm;
    }

    /* 7. Tổng hợp tín hiệu điều khiển */
    float total_out = ctrl->ff_term + ctrl->p_term + ctrl->i_term + deadzone;

    /* 8. Bão hòa đầu ra PWM [-out_max, out_max] */
    if (total_out >= ctrl->cfg.out_max) {
        ctrl->output_pwm = ctrl->cfg.out_max;
        ctrl->is_saturated = true;
    } else if (total_out <= ctrl->cfg.out_min) {
        ctrl->output_pwm = ctrl->cfg.out_min;
        ctrl->is_saturated = true;
    } else {
        ctrl->output_pwm = total_out;
        ctrl->is_saturated = false;
    }

    ctrl->prev_error = error;

    return (int16_t)roundf(ctrl->output_pwm);
}

