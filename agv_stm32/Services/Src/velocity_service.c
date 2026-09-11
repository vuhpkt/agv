/**
 * @file velocity_service.c
 * @brief Triển khai dịch vụ điều khiển vận tốc bánh xe AGV
 */

#include "velocity_service.h"
#include <stddef.h>

float velocity_service_ticks_to_rpm(uint16_t cpr, int16_t delta_ticks, float dt_s) {
    if (dt_s <= 0.0f || cpr == 0) {
        return 0.0f;
    }
    return ((float)delta_ticks * 60.0f) / ((float)cpr * dt_s);
}

void velocity_service_init(velocity_channel_t *ch, uint16_t cpr, float filter_alpha, const motor_pi_cfg_t *pi_cfg) {
    if (ch == NULL) return;

    ch->cfg.cpr = cpr;
    ch->cfg.filter_alpha = filter_alpha;

    if (pi_cfg != NULL) {
        ch->cfg.pi_cfg = *pi_cfg;
    } else {
        motor_pi_get_default_cfg(&ch->cfg.pi_cfg);
    }

    motor_pi_init(&ch->pi, &ch->cfg.pi_cfg);
    filter_lpf_init(&ch->filter, filter_alpha);

    ch->target_rpm = 0.0f;
    ch->raw_rpm = 0.0f;
    ch->filtered_rpm = 0.0f;
    ch->output_pwm = 0;
    ch->is_enabled = false;
}

void velocity_service_set_target(velocity_channel_t *ch, float target_rpm) {
    if (ch == NULL) return;
    ch->target_rpm = target_rpm;
    ch->is_enabled = true;
}

int16_t velocity_service_update(velocity_channel_t *ch, int16_t delta_ticks, float dt_s) {
    if (ch == NULL || dt_s <= 0.0f) return 0;

    /* 1. Tính tốc độ thô RPM */
    ch->raw_rpm = velocity_service_ticks_to_rpm(ch->cfg.cpr, delta_ticks, dt_s);

    /* 2. Lọc thông thấp khử nhiễu lượng tử hóa */
    ch->filtered_rpm = filter_lpf_update(&ch->filter, ch->raw_rpm);

    /* 3. Tính toán bộ điều khiển PI vòng kín nếu đang kích hoạt */
    if (ch->is_enabled) {
        ch->output_pwm = motor_pi_update(&ch->pi, ch->target_rpm, ch->filtered_rpm, dt_s);
    } else {
        ch->output_pwm = 0;
    }

    return ch->output_pwm;
}

void velocity_service_reset(velocity_channel_t *ch) {
    if (ch == NULL) return;
    ch->target_rpm = 0.0f;
    ch->raw_rpm = 0.0f;
    ch->filtered_rpm = 0.0f;
    ch->output_pwm = 0;
    ch->is_enabled = false;
    motor_pi_reset(&ch->pi);
}

void velocity_service_set_gains(velocity_channel_t *ch, float kp, float ki, float k_ff) {
    if (ch == NULL) return;
    motor_pi_set_gains(&ch->pi, kp, ki, k_ff);
}
