#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Cấu hình cơ khí và động học AGV Vi Sai (Differential Drive)
 */
typedef struct {
    float wheel_base;        /**< Khoảng cách giữa 2 tâm bánh xe (đơn vị: mét) */
    float wheel_radius;      /**< Bán kính bánh xe (đơn vị: mét) */
    float ticks_per_rev;     /**< Số xung encoder trên 1 vòng quay bánh xe */
} kinematics_cfg_t;

/**
 * @brief Vận tốc bánh xe trái/phải
 */
typedef struct {
    float left;              /**< Vận tốc bánh trái (m/s hoặc rad/s hoặc xung/s) */
    float right;             /**< Vận tốc bánh phải (m/s hoặc rad/s hoặc xung/s) */
} wheel_speeds_t;

/**
 * @brief Tọa độ và tư thế robot (Pose 2D)
 */
typedef struct {
    float x;                 /**< Tọa độ X (mét) */
    float y;                 /**< Tọa độ Y (mét) */
    float theta;             /**< Góc hướng Yaw (radian, [-pi, pi]) */
} robot_pose_t;

/**
 * @brief Handle quản lý động học và định vị Odometry của AGV
 */
typedef struct {
    kinematics_cfg_t cfg;    /**< Cấu hình kích thước xe */
    robot_pose_t pose;       /**< Tọa độ odometry hiện tại */
} kinematics_t;

/**
 * @brief Khởi tạo module động học AGV
 * @param kin Con trỏ tới kinematics handle
 * @param cfg Con trỏ tới struct cấu hình kích thước xe
 */
void kinematics_init(kinematics_t *kin, const kinematics_cfg_t *cfg);

/**
 * @brief Động học ngược (Inverse Kinematics): Từ (v, w) tính ra vận tốc 2 bánh (v_left, v_right)
 * @param kin Con trỏ tới kinematics handle
 * @param linear_vel Vận tốc dài tịnh tiến của thân xe (m/s)
 * @param angular_vel Vận tốc góc quay thân xe (rad/s)
 * @param out_speeds Con trỏ struct chứa kết quả vận tốc bánh trái / phải (m/s)
 */
void kinematics_inverse(const kinematics_t *kin, float linear_vel, float angular_vel, wheel_speeds_t *out_speeds);

/**
 * @brief Động học thuận (Forward Kinematics): Từ vận tốc 2 bánh tính ra (v, w) của thân xe
 * @param kin Con trỏ tới kinematics handle
 * @param speeds Vận tốc bánh trái / phải (m/s)
 * @param out_linear_vel Con trỏ nhận vận tốc dài (m/s)
 * @param out_angular_vel Con trỏ nhận vận tốc góc (rad/s)
 */
void kinematics_forward(const kinematics_t *kin, const wheel_speeds_t *speeds, float *out_linear_vel, float *out_angular_vel);

/**
 * @brief Cập nhật tọa độ Odometry dựa trên quãng đường di chuyển của 2 bánh
 * @param kin Con trỏ tới kinematics handle
 * @param delta_dist_left Quãng đường bánh trái di chuyển trong chu kỳ vừa qua (mét)
 * @param delta_dist_right Quãng đường bánh phải di chuyển trong chu kỳ vừa qua (mét)
 */
void kinematics_update_odometry(kinematics_t *kin, float delta_dist_left, float delta_dist_right);

/**
 * @brief Đặt lại tọa độ Odometry về vị trí tùy chọn
 * @param kin Con trỏ tới kinematics handle
 * @param x Tọa độ X ban đầu (mét)
 * @param y Tọa độ Y ban đầu (mét)
 * @param theta Góc hướng ban đầu (radian)
 */
void kinematics_reset_pose(kinematics_t *kin, float x, float y, float theta);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */

