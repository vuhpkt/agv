#ifndef TB6612_H
#define TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief TB6612 Motor Operating Mode
 */
typedef enum {
    TB6612_MODE_COAST = 0,    /**< Dừng trôi: IN1=0, IN2=0, PWM=0 */
    TB6612_MODE_FORWARD,      /**< Quay thuận: IN1=1, IN2=0, PWM=duty */
    TB6612_MODE_BACKWARD,     /**< Quay nghịch: IN1=0, IN2=1, PWM=duty */
    TB6612_MODE_BRAKE,        /**< Phanh chủ động (Short brake): IN1=1, IN2=1, PWM=0 */
    TB6612_MODE_STANDBY       /**< Chế độ chờ tiết kiệm năng lượng: STBY=0 */
} tb6612_mode_t;

/**
 * @brief Callback function prototypes for Hardware I/O Binding
 */
typedef void (*tb6612_set_pin_fn)(bool state, void *user_data);
typedef void (*tb6612_set_pwm_fn)(uint16_t duty, void *user_data);

/**
 * @brief Hardware I/O Interface for a single TB6612 channel
 */
typedef struct {
    tb6612_set_pin_fn set_in1;   /**< Callback điều khiển chân IN1 */
    tb6612_set_pin_fn set_in2;   /**< Callback điều khiển chân IN2 */
    tb6612_set_pwm_fn set_pwm;   /**< Callback thiết lập độ rộng xung PWM */
    tb6612_set_pin_fn set_stby;  /**< Callback điều khiển chân STBY (tùy chọn, có thể NULL) */
} tb6612_io_t;

/**
 * @brief Static configuration for TB6612 channel
 */
typedef struct {
    uint16_t max_pwm;            /**< Giá trị PWM cực đại (ví dụ 1000) */
    bool inverted;               /**< Đảo chiều quay logic (true nếu dây động cơ bị ngược) */
} tb6612_cfg_t;

/**
 * @brief TB6612 Channel Handle (Runtime State)
 */
typedef struct {
    tb6612_cfg_t cfg;
    tb6612_io_t io;
    void *user_data;             /**< Con trỏ dữ liệu tùy chọn truyền vào callbacks */
    tb6612_mode_t mode;          /**< Chế độ hoạt động hiện tại */
    int16_t current_speed;       /**< Tốc độ/PWM hiện tại [-max_pwm, max_pwm] */
    bool is_standby;             /**< Trạng thái standby */
} tb6612_t;

/**
 * @brief Khởi tạo handle điều khiển TB6612
 * @param handle Con trỏ tới tb6612 handle
 * @param cfg Con trỏ tới struct cấu hình tĩnh (nếu NULL sẽ dùng cấu hình mặc định)
 * @param io Con trỏ tới struct callbacks IO phần cứng
 * @param user_data Con trỏ dữ liệu tùy chọn truyền vào callbacks
 */
void tb6612_init(tb6612_t *handle, const tb6612_cfg_t *cfg, const tb6612_io_t *io, void *user_data);

/**
 * @brief Thiết lập tốc độ và chiều quay cho động cơ
 * @param handle Con trỏ tới tb6612 handle
 * @param speed Tốc độ từ -max_pwm đến +max_pwm (>0: Forward, <0: Backward, =0: Coast)
 */
void tb6612_set_speed(tb6612_t *handle, int16_t speed);

/**
 * @brief Dừng trôi động cơ (Coast / High impedance)
 * @param handle Con trỏ tới tb6612 handle
 */
void tb6612_coast(tb6612_t *handle);

/**
 * @brief Phanh chủ động động cơ (Short Brake)
 * @param handle Con trỏ tới tb6612 handle
 */
void tb6612_brake(tb6612_t *handle);

/**
 * @brief Điều khiển chế độ Standby của IC
 * @param handle Con trỏ tới tb6612 handle
 * @param enable_standby true: Kích hoạt Standby (tắt cầu H), false: Hoạt động bình thường
 */
void tb6612_set_standby(tb6612_t *handle, bool enable_standby);

/**
 * @brief Lấy tốc độ hiện tại của động cơ
 * @param handle Con trỏ tới tb6612 handle
 * @return int16_t Giá trị tốc độ hiện tại
 */
int16_t tb6612_get_speed(const tb6612_t *handle);

/**
 * @brief Lấy chế độ hoạt động hiện tại
 * @param handle Con trỏ tới tb6612 handle
 * @return tb6612_mode_t Chế độ hoạt động
 */
tb6612_mode_t tb6612_get_mode(const tb6612_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_H */

