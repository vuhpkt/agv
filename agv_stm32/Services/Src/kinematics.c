#include "kinematics.h"
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float normalize_angle(float angle) {
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

void kinematics_init(kinematics_t *kin, const kinematics_cfg_t *cfg) {
    if (kin == NULL) {
        return;
    }

    if (cfg != NULL) {
        kin->cfg = *cfg;
    } else {
        kin->cfg.wheel_base = 0.15f;    /* 150 mm */
        kin->cfg.wheel_radius = 0.0325f; /* 65 mm diameter */
        kin->cfg.ticks_per_rev = 1320.0f;
    }

    kinematics_reset_pose(kin, 0.0f, 0.0f, 0.0f);
}

void kinematics_inverse(const kinematics_t *kin, float linear_vel, float angular_vel, wheel_speeds_t *out_speeds) {
    if (kin == NULL || out_speeds == NULL) {
        return;
    }

    float half_base = kin->cfg.wheel_base * 0.5f;
    out_speeds->left = linear_vel - (angular_vel * half_base);
    out_speeds->right = linear_vel + (angular_vel * half_base);
}

void kinematics_forward(const kinematics_t *kin, const wheel_speeds_t *speeds, float *out_linear_vel, float *out_angular_vel) {
    if (kin == NULL || speeds == NULL) {
        return;
    }

    if (out_linear_vel) {
        *out_linear_vel = (speeds->right + speeds->left) * 0.5f;
    }

    if (out_angular_vel) {
        if (kin->cfg.wheel_base > 0.0001f) {
            *out_angular_vel = (speeds->right - speeds->left) / kin->cfg.wheel_base;
        } else {
            *out_angular_vel = 0.0f;
        }
    }
}

void kinematics_update_odometry(kinematics_t *kin, float delta_dist_left, float delta_dist_right) {
    if (kin == NULL || kin->cfg.wheel_base <= 0.0001f) {
        return;
    }

    /* Quãng đường trung bình và góc xoay thay đổi */
    float delta_distance = (delta_dist_right + delta_dist_left) * 0.5f;
    float delta_theta = (delta_dist_right - delta_dist_left) / kin->cfg.wheel_base;

    /* Tích hợp vị trí theo mô hình Runge-Kutta bậc 2 (Midpoint arc integration) */
    float avg_theta = kin->pose.theta + (delta_theta * 0.5f);

    kin->pose.x += delta_distance * cosf(avg_theta);
    kin->pose.y += delta_distance * sinf(avg_theta);
    kin->pose.theta = normalize_angle(kin->pose.theta + delta_theta);
}

void kinematics_reset_pose(kinematics_t *kin, float x, float y, float theta) {
    if (kin == NULL) {
        return;
    }

    kin->pose.x = x;
    kin->pose.y = y;
    kin->pose.theta = normalize_angle(theta);
}

