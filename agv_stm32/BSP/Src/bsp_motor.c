#include "bsp_motor.h"
#include "tim.h"
#include <stdlib.h>

static tb6612_t s_motor_left;
static tb6612_t s_motor_right;

/* Callbacks phần cứng cho Động cơ Trái (Kênh A) */
static void motor_left_set_in1(bool state, void *user_data) {
    (void)user_data;
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void motor_left_set_in2(bool state, void *user_data) {
    (void)user_data;
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void motor_left_set_pwm(uint16_t duty, void *user_data) {
    (void)user_data;
    /* Ánh xạ duty [0, BSP_MOTOR_MAX_SPEED] sang dải xung thực tế của TIM4 [0, Period] */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t pulse = ((uint32_t)duty * period) / BSP_MOTOR_MAX_SPEED;
    if (pulse > period) pulse = period;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint16_t)pulse);
}

/* Callbacks phần cứng cho Động cơ Phải (Kênh B) */
static void motor_right_set_in1(bool state, void *user_data) {
    (void)user_data;
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void motor_right_set_in2(bool state, void *user_data) {
    (void)user_data;
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void motor_right_set_pwm(uint16_t duty, void *user_data) {
    (void)user_data;
    /* Ánh xạ duty [0, BSP_MOTOR_MAX_SPEED] sang dải xung thực tế của TIM4 [0, Period] */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t pulse = ((uint32_t)duty * period) / BSP_MOTOR_MAX_SPEED;
    if (pulse > period) pulse = period;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)pulse);
}

/* Callback phần cứng cho chân STBY dùng chung */
static void motor_set_stby(bool state, void *user_data) {
    (void)user_data;
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_Motor_Init(void) {
    /* Bật clock AFIO và tắt JTAG (giữ SWD PA13/PA14) để giải phóng PB3, PB4, PA15 làm GPIO cho TB6612 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* Khởi động PWM cho Channel 2 (PWMA - PB7) và Channel 3 (PWMB - PB8) của TIM4 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    /* Thiết lập duty cycle ban đầu = 0 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

    /* Cấu hình IO & Config cho Động cơ Trái */
    tb6612_cfg_t cfg_left = {
        .max_pwm = BSP_MOTOR_MAX_SPEED,
        .inverted = true
    };
    tb6612_io_t io_left = {
        .set_in1 = motor_left_set_in1,
        .set_in2 = motor_left_set_in2,
        .set_pwm = motor_left_set_pwm,
        .set_stby = motor_set_stby
    };
    tb6612_init(&s_motor_left, &cfg_left, &io_left, NULL);

    /* Cấu hình IO & Config cho Động cơ Phải */
    tb6612_cfg_t cfg_right = {
        .max_pwm = BSP_MOTOR_MAX_SPEED,
        .inverted = false
    };
    tb6612_io_t io_right = {
        .set_in1 = motor_right_set_in1,
        .set_in2 = motor_right_set_in2,
        .set_pwm = motor_right_set_pwm,
        .set_stby = motor_set_stby
    };
    tb6612_init(&s_motor_right, &cfg_right, &io_right, NULL);

    /* Thoát chế độ Standby của TB6612 */
    BSP_Motor_Standby(false);

    /* Dừng cả 2 động cơ */
    BSP_Motor_Stop(MOTOR_LEFT);
    BSP_Motor_Stop(MOTOR_RIGHT);
}

void BSP_Motor_SetSpeed(Motor_ID_t motor, int16_t speed) {
    if (motor == MOTOR_LEFT) {
        tb6612_set_speed(&s_motor_left, speed);
    } else if (motor == MOTOR_RIGHT) {
        tb6612_set_speed(&s_motor_right, speed);
    }
}

void BSP_Motor_SetSpeeds(int16_t left_speed, int16_t right_speed) {
    BSP_Motor_SetSpeed(MOTOR_LEFT, left_speed);
    BSP_Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}

void BSP_Motor_Stop(Motor_ID_t motor) {
    if (motor == MOTOR_LEFT) {
        tb6612_coast(&s_motor_left);
    } else if (motor == MOTOR_RIGHT) {
        tb6612_coast(&s_motor_right);
    }
}

void BSP_Motor_Brake(Motor_ID_t motor) {
    if (motor == MOTOR_LEFT) {
        tb6612_brake(&s_motor_left);
    } else if (motor == MOTOR_RIGHT) {
        tb6612_brake(&s_motor_right);
    }
}

void BSP_Motor_Standby(bool enable_standby) {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, enable_standby ? GPIO_PIN_RESET : GPIO_PIN_SET);
    s_motor_left.is_standby = enable_standby;
    s_motor_right.is_standby = enable_standby;
}

tb6612_t* BSP_Motor_GetHandle(Motor_ID_t motor) {
    return (motor == MOTOR_LEFT) ? &s_motor_left : &s_motor_right;
}
