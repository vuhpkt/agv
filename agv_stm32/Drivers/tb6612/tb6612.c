#include "tb6612.h"
#include <stdlib.h>

void tb6612_init(tb6612_t *handle, const tb6612_cfg_t *cfg, const tb6612_io_t *io, void *user_data) {
    if (handle == NULL) {
        return;
    }

    if (cfg != NULL) {
        handle->cfg = *cfg;
    } else {
        handle->cfg.max_pwm = 1000;
        handle->cfg.inverted = false;
    }

    if (io != NULL) {
        handle->io = *io;
    } else {
        handle->io.set_in1 = NULL;
        handle->io.set_in2 = NULL;
        handle->io.set_pwm = NULL;
        handle->io.set_stby = NULL;
    }

    handle->user_data = user_data;
    handle->mode = TB6612_MODE_COAST;
    handle->current_speed = 0;
    handle->is_standby = false;

    /* Thoát Standby nếu có cấu hình callback */
    tb6612_set_standby(handle, false);

    /* Đưa động cơ về trạng thái dừng trôi ban đầu */
    tb6612_coast(handle);
}

void tb6612_set_speed(tb6612_t *handle, int16_t speed) {
    if (handle == NULL) {
        return;
    }

    /* Bão hòa tốc độ trong dải [-max_pwm, max_pwm] */
    if (speed > (int16_t)handle->cfg.max_pwm) {
        speed = (int16_t)handle->cfg.max_pwm;
    } else if (speed < -(int16_t)handle->cfg.max_pwm) {
        speed = -(int16_t)handle->cfg.max_pwm;
    }

    handle->current_speed = speed;

    /* Xử lý cờ đảo chiều logic */
    int16_t effective_speed = handle->cfg.inverted ? -speed : speed;
    uint16_t duty = (uint16_t)abs(effective_speed);

    if (effective_speed > 0) {
        /* Quay thuận: IN1 = HIGH, IN2 = LOW */
        handle->mode = TB6612_MODE_FORWARD;
        if (handle->io.set_in1) handle->io.set_in1(true, handle->user_data);
        if (handle->io.set_in2) handle->io.set_in2(false, handle->user_data);
        if (handle->io.set_pwm) handle->io.set_pwm(duty, handle->user_data);
    } else if (effective_speed < 0) {
        /* Quay nghịch: IN1 = LOW, IN2 = HIGH */
        handle->mode = TB6612_MODE_BACKWARD;
        if (handle->io.set_in1) handle->io.set_in1(false, handle->user_data);
        if (handle->io.set_in2) handle->io.set_in2(true, handle->user_data);
        if (handle->io.set_pwm) handle->io.set_pwm(duty, handle->user_data);
    } else {
        /* Tốc độ 0: Dừng trôi */
        tb6612_coast(handle);
    }
}

void tb6612_coast(tb6612_t *handle) {
    if (handle == NULL) {
        return;
    }

    handle->mode = TB6612_MODE_COAST;
    handle->current_speed = 0;

    if (handle->io.set_in1) handle->io.set_in1(false, handle->user_data);
    if (handle->io.set_in2) handle->io.set_in2(false, handle->user_data);
    if (handle->io.set_pwm) handle->io.set_pwm(0, handle->user_data);
}

void tb6612_brake(tb6612_t *handle) {
    if (handle == NULL) {
        return;
    }

    handle->mode = TB6612_MODE_BRAKE;
    handle->current_speed = 0;

    /* Short Brake: IN1 = HIGH, IN2 = HIGH, PWM = 0 */
    if (handle->io.set_pwm) handle->io.set_pwm(0, handle->user_data);
    if (handle->io.set_in1) handle->io.set_in1(true, handle->user_data);
    if (handle->io.set_in2) handle->io.set_in2(true, handle->user_data);
}

void tb6612_set_standby(tb6612_t *handle, bool enable_standby) {
    if (handle == NULL) {
        return;
    }

    handle->is_standby = enable_standby;
    if (enable_standby) {
        handle->mode = TB6612_MODE_STANDBY;
    }

    if (handle->io.set_stby) {
        /* Chân STBY mức tích cực HIGH: true (Standby) -> Pin LOW, false (Normal) -> Pin HIGH */
        handle->io.set_stby(!enable_standby, handle->user_data);
    }
}

int16_t tb6612_get_speed(const tb6612_t *handle) {
    return handle ? handle->current_speed : 0;
}

tb6612_mode_t tb6612_get_mode(const tb6612_t *handle) {
    return handle ? handle->mode : TB6612_MODE_COAST;
}

