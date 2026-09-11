#include "filter.h"
#include <stddef.h>

void filter_lpf_init(filter_lpf_t *filter, float alpha) {
    if (filter == NULL) {
        return;
    }

    if (alpha <= 0.0f) {
        alpha = 0.1f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    filter->alpha = alpha;
    filter->prev_output = 0.0f;
    filter->is_initialized = false;
}

float filter_lpf_update(filter_lpf_t *filter, float input) {
    if (filter == NULL) {
        return input;
    }

    if (!filter->is_initialized) {
        filter->prev_output = input;
        filter->is_initialized = true;
        return input;
    }

    /* Công thức Exponential Moving Average: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
    float output = (filter->alpha * input) + ((1.0f - filter->alpha) * filter->prev_output);
    filter->prev_output = output;
    return output;
}

void filter_lpf_reset(filter_lpf_t *filter) {
    if (filter == NULL) {
        return;
    }

    filter->prev_output = 0.0f;
    filter->is_initialized = false;
}

