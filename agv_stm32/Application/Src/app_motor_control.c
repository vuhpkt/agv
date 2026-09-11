/**
 * @file app_motor_control.c
 * @brief Triển khai module điều khiển vận tốc kín 2 bánh xe AGV & Multi-Speed Test
 */

#include "app_motor_control.h"
#include "app_config.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_board.h"
#include "velocity_service.h"
#include "app_comms.h"

#if AGV_ENABLE_MOTOR_TEST_MODE
#include "diag_motor.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MOTOR_CONTROL_RATE_MS     10      /**< Chu kỳ điều khiển vận tốc 10ms (100Hz) */
#define MOTOR_ENCODER_CPR         1320    /**< Số xung / vòng của encoder */
#define MOTOR_FILTER_ALPHA        0.85f   /**< Hệ số lọc thông thấp tốc độ đo */

static velocity_channel_t s_vel_left;
static velocity_channel_t s_vel_right;
static uint32_t s_last_control_tick = 0;

void App_MotorControl_Init(void) {
    velocity_service_init(&s_vel_left, MOTOR_ENCODER_CPR, MOTOR_FILTER_ALPHA, NULL);
    velocity_service_init(&s_vel_right, MOTOR_ENCODER_CPR, MOTOR_FILTER_ALPHA, NULL);
    s_last_control_tick = bsp_time_get_ms();
}

void App_MotorControl_SetTargetRPM(float left_rpm, float right_rpm) {
    velocity_service_set_target(&s_vel_left, left_rpm);
    velocity_service_set_target(&s_vel_right, right_rpm);
}

void App_MotorControl_GetSpeeds(float *left_rpm, float *right_rpm) {
    if (left_rpm) *left_rpm = s_vel_left.filtered_rpm;
    if (right_rpm) *right_rpm = s_vel_right.filtered_rpm;
}

void App_MotorControl_GetTargets(float *left_rpm, float *right_rpm) {
    if (left_rpm) *left_rpm = s_vel_left.target_rpm;
    if (right_rpm) *right_rpm = s_vel_right.target_rpm;
}

void App_MotorControl_SetGains(float kp, float ki, float k_ff) {
    velocity_service_set_gains(&s_vel_left, kp, ki, k_ff);
    velocity_service_set_gains(&s_vel_right, kp, ki, k_ff);
}

void App_MotorControl_Stop(void) {
    velocity_service_reset(&s_vel_left);
    velocity_service_reset(&s_vel_right);
    BSP_Motor_Stop(MOTOR_LEFT);
    BSP_Motor_Stop(MOTOR_RIGHT);
}

bool App_MotorControl_IsEnabled(void) {
    return s_vel_left.is_enabled || s_vel_right.is_enabled;
}

void App_MotorControl_PrintStatus(void) {
    int sp_l_int = (int)s_vel_left.target_rpm;
    int sp_l_dec = abs((int)(s_vel_left.target_rpm * 10.0f)) % 10;
    int sp_r_int = (int)s_vel_right.target_rpm;
    int sp_r_dec = abs((int)(s_vel_right.target_rpm * 10.0f)) % 10;

    int act_l_int = (int)s_vel_left.filtered_rpm;
    int act_l_dec = abs((int)(s_vel_left.filtered_rpm * 10.0f)) % 10;
    int act_r_int = (int)s_vel_right.filtered_rpm;
    int act_r_dec = abs((int)(s_vel_right.filtered_rpm * 10.0f)) % 10;

    AGV_LOG("PI L: tgt=%d.%d act=%d.%d pwm=%d | R: tgt=%d.%d act=%d.%d pwm=%d",
            sp_l_int, sp_l_dec, act_l_int, act_l_dec, (int)s_vel_left.output_pwm,
            sp_r_int, sp_r_dec, act_r_int, act_r_dec, (int)s_vel_right.output_pwm);
}

void App_MotorControl_Process(void) {
#if AGV_ENABLE_MOTOR_TEST_MODE
    /* Nhường toàn quyền điều khiển động cơ cho bài Step Test nếu đang kích hoạt trong test mode */
    if (Diag_Motor_IsRunning()) {
        if (App_MotorControl_IsEnabled()) {
            App_MotorControl_Stop();
        }
        return;
    }
#endif

    uint32_t current_tick = bsp_time_get_ms();
    if (current_tick - s_last_control_tick < MOTOR_CONTROL_RATE_MS) {
        return;
    }

    /* Khóa pha chu kỳ 10ms */
    s_last_control_tick += MOTOR_CONTROL_RATE_MS;
    if (current_tick - s_last_control_tick > MOTOR_CONTROL_RATE_MS * 5) {
        s_last_control_tick = current_tick;
    }

    /* Đọc xung encoder */
    BSP_Encoder_Update();
    int16_t delta_l = BSP_Encoder_GetDelta(ENCODER_LEFT);
    int16_t delta_r = BSP_Encoder_GetDelta(ENCODER_RIGHT);

    float dt_s = (float)MOTOR_CONTROL_RATE_MS / 1000.0f;
    int16_t pwm_l = velocity_service_update(&s_vel_left, delta_l, dt_s);
    int16_t pwm_r = velocity_service_update(&s_vel_right, delta_r, dt_s);

    if (App_MotorControl_IsEnabled()) {
        BSP_Motor_SetSpeeds(pwm_l, pwm_r);
    }
}
