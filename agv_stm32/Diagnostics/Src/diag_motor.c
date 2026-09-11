/**
 * @file diag_motor.c
 * @brief Triển khai module kiểm tra đáp ứng xung đơn vị (Unit Step Response) động cơ AGV
 */

#include "diag_motor.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_board.h"
#include "app_comms.h"
#include "velocity_service.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI_CONST 3.14159265358979323846f

static bool s_running = false;
static Motor_ID_t s_motor = MOTOR_LEFT;
static Encoder_ID_t s_encoder = ENCODER_LEFT;
static int16_t s_target_pwm = DIAG_MOTOR_DEFAULT_PWM;
static uint32_t s_step_duration_ms = DIAG_MOTOR_DEFAULT_DURATION_MS;
static uint32_t s_total_duration_ms = DIAG_MOTOR_PRE_DELAY_MS + DIAG_MOTOR_DEFAULT_DURATION_MS + DIAG_MOTOR_POST_DELAY_MS;

static uint32_t s_last_sample_tick = 0;
static uint32_t s_sample_idx = 0;
static velocity_channel_t s_vel_chan;

static Diag_Motor_Result_t s_result;
static float s_steady_sum = 0.0f;
static uint32_t s_steady_count = 0;

/**
 * @brief Định dạng số thực có dấu sang chuỗi mà không cần newlib-nano _printf_float
 */
static void format_signed_float(char *out, size_t max_len, float val, int decimals) {
    if (max_len == 0 || out == NULL) return;
    int is_neg = (val < 0.0f) ? 1 : 0;
    float abs_v = fabsf(val);
    int32_t int_part = (int32_t)abs_v;
    float scale = 1.0f;
    for (int i = 0; i < decimals; i++) {
        scale *= 10.0f;
    }
    int32_t frac_part = (int32_t)((abs_v - (float)int_part) * scale + 0.5f);
    if (frac_part >= (int32_t)scale) {
        int_part++;
        frac_part = 0;
    }
    if (is_neg && (int_part != 0 || frac_part != 0)) {
        snprintf(out, max_len, "-%ld.%0*ld", (long)int_part, decimals, (long)frac_part);
    } else {
        snprintf(out, max_len, "%ld.%0*ld", (long)int_part, decimals, (long)frac_part);
    }
}

/**
 * @brief Gửi một dòng dữ liệu mẫu telemetry qua UART và RTT
 * Định dạng: DATA:Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS
 */
static void send_sample_data(uint32_t time_ms, int16_t pwm, int16_t delta_ticks, float speed_rpm, float speed_rads) {
    char buf[128];
    char rpm_str[24];
    char rads_str[24];
    format_signed_float(rpm_str, sizeof(rpm_str), speed_rpm, 2);
    format_signed_float(rads_str, sizeof(rads_str), speed_rads, 3);

    uint32_t time_s_int = time_ms / 1000;
    uint32_t time_s_frac = (time_ms % 1000) * 10; /* 4 chữ số phần thập phân */

    char pwm_pct_str[16];
    float pwm_pct_val = ((float)pwm * 100.0f) / (float)BSP_MOTOR_MAX_SPEED;
    format_signed_float(pwm_pct_str, sizeof(pwm_pct_str), pwm_pct_val, 1);

    int len = snprintf(buf, sizeof(buf),
                       "DATA:%lu,%lu.%04lu,%d,%s,%d,%s,%s\r\n",
                       (unsigned long)time_ms,
                       (unsigned long)time_s_int, (unsigned long)time_s_frac,
                       (int)pwm,
                       pwm_pct_str,
                       (int)delta_ticks,
                       rpm_str, rads_str);
    if (len > 0) {
        AGV_LOG("%s", buf);
    }
}

void Diag_Motor_Init(void) {
    s_running = false;
    s_sample_idx = 0;
    s_motor = MOTOR_LEFT;
    s_encoder = ENCODER_LEFT;
    s_target_pwm = DIAG_MOTOR_DEFAULT_PWM;
    s_step_duration_ms = DIAG_MOTOR_DEFAULT_DURATION_MS;
    s_total_duration_ms = DIAG_MOTOR_PRE_DELAY_MS + s_step_duration_ms + DIAG_MOTOR_POST_DELAY_MS;

    velocity_service_init(&s_vel_chan, DIAG_MOTOR_CPR, DIAG_MOTOR_FILTER_ALPHA, NULL);

    s_result.status = DIAG_MOTOR_STATE_IDLE;
    s_result.motor = MOTOR_LEFT;
    s_result.target_pwm = DIAG_MOTOR_DEFAULT_PWM;
    s_result.step_duration_ms = DIAG_MOTOR_DEFAULT_DURATION_MS;
    s_result.peak_speed_rpm = 0.0f;
    s_result.steady_speed_rpm = 0.0f;
    s_result.total_encoder_ticks = 0;
    s_result.total_samples = 0;
    s_result.encoder_detected = false;
    s_result.speed_reached = false;
}

void Diag_Motor_Start(Motor_ID_t motor, int16_t target_pwm, uint32_t step_duration_ms) {
    /* 1. Dừng cả hai động cơ an toàn trước khi test */
    BSP_Motor_Stop(MOTOR_LEFT);
    BSP_Motor_Stop(MOTOR_RIGHT);

    /* 2. Thiết lập tham số test */
    s_motor = motor;
    s_encoder = (motor == MOTOR_LEFT) ? ENCODER_LEFT : ENCODER_RIGHT;
    s_target_pwm = target_pwm;
    s_step_duration_ms = (step_duration_ms > 0) ? step_duration_ms : DIAG_MOTOR_DEFAULT_DURATION_MS;
    s_total_duration_ms = DIAG_MOTOR_PRE_DELAY_MS + s_step_duration_ms + DIAG_MOTOR_POST_DELAY_MS;
    s_sample_idx = 0;

    /* 3. Khởi tạo theo dõi kết quả unit test */
    s_result.status = DIAG_MOTOR_STATE_RUNNING;
    s_result.motor = motor;
    s_result.target_pwm = target_pwm;
    s_result.step_duration_ms = s_step_duration_ms;
    s_result.peak_speed_rpm = 0.0f;
    s_result.steady_speed_rpm = 0.0f;
    s_result.total_encoder_ticks = 0;
    s_result.total_samples = 0;
    s_result.encoder_detected = false;
    s_result.speed_reached = false;
    s_steady_sum = 0.0f;
    s_steady_count = 0;

    /* 4. Reset encoder và bộ lọc */
    BSP_Encoder_Reset(s_encoder);
    velocity_service_init(&s_vel_chan, DIAG_MOTOR_CPR, DIAG_MOTOR_FILTER_ALPHA, NULL);

    s_last_sample_tick = bsp_time_get_ms();

    s_running = true;

    char banner_pct_str[16];
    float banner_pct_val = ((float)s_target_pwm * 100.0f) / (float)BSP_MOTOR_MAX_SPEED;
    format_signed_float(banner_pct_str, sizeof(banner_pct_str), banner_pct_val, 1);

    AGV_LOG("[STEP_TEST_START] Motor: %s | PWM: %d (%s%%) | Dur: %u ms",
            (s_motor == MOTOR_LEFT) ? "LEFT" : "RIGHT",
            (int)s_target_pwm, banner_pct_str, (unsigned)s_step_duration_ms);
}

void Diag_Motor_StartDefault(void) {
    Diag_Motor_Start(MOTOR_LEFT, DIAG_MOTOR_DEFAULT_PWM, DIAG_MOTOR_DEFAULT_DURATION_MS);
}

void Diag_Motor_Stop(void) {
    if (s_running) {
        s_running = false;
        BSP_Motor_Stop(s_motor);
    }
}

bool Diag_Motor_IsRunning(void) {
    return s_running;
}

const Diag_Motor_Result_t* Diag_Motor_GetLastResult(void) {
    return &s_result;
}

void Diag_Motor_Process(void) {
    if (!s_running) {
        return;
    }

    uint32_t current_tick = bsp_time_get_ms();
    if (current_tick - s_last_sample_tick < DIAG_MOTOR_SAMPLE_RATE_MS) {
        return;
    }

    /* Khóa pha chu kỳ lấy mẫu 10ms (100Hz) để triệt tiêu sai số trôi tích lũy */
    s_last_sample_tick += DIAG_MOTOR_SAMPLE_RATE_MS;
    if (current_tick - s_last_sample_tick > DIAG_MOTOR_SAMPLE_RATE_MS * 5) {
        s_last_sample_tick = current_tick;
    }

    uint32_t elapsed_ms = s_sample_idx * DIAG_MOTOR_SAMPLE_RATE_MS;
    int16_t current_pwm = 0;

    if (elapsed_ms < DIAG_MOTOR_PRE_DELAY_MS) {
        /* Giai đoạn 1: Baseline 0ms - 200ms (PWM = 0) */
        current_pwm = 0;
    } else if (elapsed_ms < (DIAG_MOTOR_PRE_DELAY_MS + s_step_duration_ms)) {
        /* Giai đoạn 2: Kích xung đơn vị PWM (ví dụ 80% = 800) */
        current_pwm = s_target_pwm;
    } else if (elapsed_ms < s_total_duration_ms) {
        /* Giai đoạn 3: Ngắt PWM (Coast/Thả trôi 500ms) để quan sát quán tính giảm tốc */
        current_pwm = 0;
    } else {
        /* Giai đoạn 4: Hoàn tất bài test và đánh giá PASS/FAIL */
        Diag_Motor_Stop();

        if (s_steady_count > 0) {
            s_result.steady_speed_rpm = s_steady_sum / (float)s_steady_count;
        }
        s_result.total_samples = s_sample_idx;

        /*
         * Tiêu chí đánh giá Unit Test:
         * 1. Phải ghi nhận chuyển động encoder (> 20 ticks)
         * 2. Vận tốc đỉnh phải đạt ít nhất 30% tốc độ kỳ vọng tỷ lệ theo PWM
         */
        float expected_min_rpm = fabsf((float)s_target_pwm / (float)BSP_MOTOR_MAX_SPEED) * 50.0f;
        s_result.encoder_detected = (labs(s_result.total_encoder_ticks) > 20);
        s_result.speed_reached = (fabsf(s_result.peak_speed_rpm) >= expected_min_rpm);

        if (s_result.encoder_detected && s_result.speed_reached) {
            s_result.status = DIAG_MOTOR_STATE_PASSED;
        } else {
            s_result.status = DIAG_MOTOR_STATE_FAILED;
        }

        char peak_str[24];
        char ss_str[24];
        format_signed_float(peak_str, sizeof(peak_str), s_result.peak_speed_rpm, 1);
        format_signed_float(ss_str, sizeof(ss_str), s_result.steady_speed_rpm, 1);

        AGV_LOG("[STEP_TEST] %s | Peak: %s RPM | Steady: %s RPM | Ticks: %ld",
                (s_result.status == DIAG_MOTOR_STATE_PASSED) ? "PASS" : "FAIL",
                peak_str, ss_str, (long)s_result.total_encoder_ticks);

        /* Kiểm tra tương quan chiều quay PWM vs Ticks Encoder */
        bool dir_match = false;
        if ((s_target_pwm > 0 && s_result.total_encoder_ticks > 0) ||
            (s_target_pwm < 0 && s_result.total_encoder_ticks < 0) ||
            (s_target_pwm == 0)) {
            dir_match = true;
        }

        if (dir_match) {
            AGV_LOG("[DIR_CHECK] PASS: PWM (%d) & Ticks (%ld) MATCH", (int)s_target_pwm, (long)s_result.total_encoder_ticks);
        } else {
            AGV_LOG("[DIR_CHECK] WARN: MISMATCH! PWM=%d Ticks=%ld", (int)s_target_pwm, (long)s_result.total_encoder_ticks);
        }
        return;
    }

    /* Xuất PWM điều khiển động cơ */
    BSP_Motor_SetSpeed(s_motor, current_pwm);

    /* Cập nhật encoder đọc delta xung */
    BSP_Encoder_Update();
    int16_t delta = BSP_Encoder_GetDelta(s_encoder);
    s_result.total_encoder_ticks += delta;

    /* Tính tốc độ qua velocity_service */
    float dt_s = (float)DIAG_MOTOR_SAMPLE_RATE_MS / 1000.0f;
    velocity_service_update(&s_vel_chan, delta, dt_s);
    float speed_rpm = s_vel_chan.filtered_rpm;

    /* Cập nhật cực đại vận tốc */
    if (fabsf(speed_rpm) > fabsf(s_result.peak_speed_rpm)) {
        s_result.peak_speed_rpm = speed_rpm;
    }

    /* Tính trung bình vùng xác lập (40% cuối của giai đoạn kích xung) */
    uint32_t step_start = DIAG_MOTOR_PRE_DELAY_MS;
    uint32_t step_end = DIAG_MOTOR_PRE_DELAY_MS + s_step_duration_ms;
    uint32_t steady_start = step_start + (s_step_duration_ms * 60 / 100);
    if (elapsed_ms >= steady_start && elapsed_ms < step_end) {
        s_steady_sum += speed_rpm;
        s_steady_count++;
    }

    /* Đổi sang rad/s: omega = rpm * (2*pi / 60) */
    float speed_rads = speed_rpm * (2.0f * PI_CONST / 60.0f);

    /* Gửi dữ liệu telemetry */
    send_sample_data(elapsed_ms, current_pwm, delta, speed_rpm, speed_rads);

    s_sample_idx++;
}
