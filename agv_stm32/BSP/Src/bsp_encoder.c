#include "bsp_encoder.h"
#include "tim.h"

static encoder_t s_enc_left;
static encoder_t s_enc_right;

/* Callback đọc Timer 2 (Encoder Trái) */
static uint32_t enc_left_get_raw(void *user_data) {
    (void)user_data;
    return (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void enc_left_reset_raw(void *user_data) {
    (void)user_data;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/* Callback đọc Timer 3 (Encoder Phải) */
static uint32_t enc_right_get_raw(void *user_data) {
    (void)user_data;
    return (uint32_t)__HAL_TIM_GET_COUNTER(&htim3);
}

static void enc_right_reset_raw(void *user_data) {
    (void)user_data;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void BSP_Encoder_Init(void) {
    /* Khởi động bộ đếm phần cứng TIM2 và TIM3 ở chế độ Encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* Cấu hình IO & Config cho Encoder Trái */
    encoder_cfg_t cfg_left = {
        .inverted = false,
        .cpr = 1320
    };
    encoder_io_t io_left = {
        .get_raw_count = enc_left_get_raw,
        .reset_raw_count = enc_left_reset_raw
    };
    encoder_init(&s_enc_left, &cfg_left, &io_left, NULL);

    /* Cấu hình IO & Config cho Encoder Phải */
    encoder_cfg_t cfg_right = {
        .inverted = false,
        .cpr = 1320
    };
    encoder_io_t io_right = {
        .get_raw_count = enc_right_get_raw,
        .reset_raw_count = enc_right_reset_raw
    };
    encoder_init(&s_enc_right, &cfg_right, &io_right, NULL);
}

void BSP_Encoder_Update(void) {
    encoder_update(&s_enc_left);
    encoder_update(&s_enc_right);
}

int32_t BSP_Encoder_GetCount(Encoder_ID_t encoder) {
    return (encoder == ENCODER_LEFT) ? encoder_get_count(&s_enc_left) : encoder_get_count(&s_enc_right);
}

int16_t BSP_Encoder_GetDelta(Encoder_ID_t encoder) {
    return (encoder == ENCODER_LEFT) ? encoder_get_delta(&s_enc_left) : encoder_get_delta(&s_enc_right);
}

void BSP_Encoder_Reset(Encoder_ID_t encoder) {
    if (encoder == ENCODER_LEFT) {
        encoder_reset(&s_enc_left);
    } else if (encoder == ENCODER_RIGHT) {
        encoder_reset(&s_enc_right);
    }
}

void BSP_Encoder_ResetAll(void) {
    encoder_reset(&s_enc_left);
    encoder_reset(&s_enc_right);
}

encoder_t* BSP_Encoder_GetHandle(Encoder_ID_t encoder) {
    return (encoder == ENCODER_LEFT) ? &s_enc_left : &s_enc_right;
}
