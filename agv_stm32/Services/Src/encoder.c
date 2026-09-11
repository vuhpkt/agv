#include "encoder.h"
#include <stddef.h>

void encoder_init(encoder_t *handle, const encoder_cfg_t *cfg, const encoder_io_t *io, void *user_data) {
    if (handle == NULL) {
        return;
    }

    if (cfg != NULL) {
        handle->cfg = *cfg;
    } else {
        handle->cfg.inverted = false;
        handle->cfg.cpr = 0;
    }

    if (io != NULL) {
        handle->io = *io;
    } else {
        handle->io.get_raw_count = NULL;
        handle->io.reset_raw_count = NULL;
    }

    handle->user_data = user_data;
    handle->last_raw_cnt = 0;
    handle->last_delta = 0;
    handle->total_count = 0;

    /* Reset bộ đếm phần cứng nếu có callback */
    if (handle->io.reset_raw_count) {
        handle->io.reset_raw_count(handle->user_data);
    }

    /* Đọc giá trị raw ban đầu */
    if (handle->io.get_raw_count) {
        handle->last_raw_cnt = (uint16_t)handle->io.get_raw_count(handle->user_data);
    }
}

void encoder_update(encoder_t *handle) {
    if (handle == NULL || handle->io.get_raw_count == NULL) {
        return;
    }

    /* Đọc thanh ghi counter 16-bit hiện tại */
    uint16_t current_raw = (uint16_t)handle->io.get_raw_count(handle->user_data);

    /* Tính toán delta có dấu - tự động xử lý tràn số 16-bit 65535 <-> 0 theo bù 2 */
    int16_t raw_delta = (int16_t)(current_raw - handle->last_raw_cnt);

    /* Áp dụng cờ đảo chiều nếu cấu hình */
    int16_t effective_delta = handle->cfg.inverted ? (int16_t)(-raw_delta) : raw_delta;

    /* Cập nhật trạng thái */
    handle->last_delta = effective_delta;
    handle->total_count += effective_delta;
    handle->last_raw_cnt = current_raw;
}

int32_t encoder_get_count(const encoder_t *handle) {
    return handle ? handle->total_count : 0;
}

int16_t encoder_get_delta(const encoder_t *handle) {
    return handle ? handle->last_delta : 0;
}

void encoder_reset(encoder_t *handle) {
    if (handle == NULL) {
        return;
    }

    if (handle->io.reset_raw_count) {
        handle->io.reset_raw_count(handle->user_data);
    }

    handle->last_raw_cnt = 0;
    handle->last_delta = 0;
    handle->total_count = 0;
}

