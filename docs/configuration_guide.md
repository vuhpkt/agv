# Hướng Dẫn Cấu Hình Hệ Thống Trong Mã Nguồn (Configuration Guide)

Tài liệu này tập hợp và hướng dẫn chi tiết vị trí các macro, hằng số và thông số kỹ thuật có thể tùy chỉnh trong firmware xe tự hành AGV STM32.

---

## 1. Bảng Tra Cứu Vị Trí Cấu Hình Nhanh

| Mục tiêu tùy chỉnh | Tham số chính | File định nghĩa | Tác động hệ thống |
| :--- | :--- | :--- | :--- |
| **Chế độ Chẩn đoán / Thi đấu** | `AGV_ENABLE_MOTOR_TEST_MODE` | [`Application/Inc/app_config.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_config.h) | Bật/tắt CLI RTT, tiết kiệm 16.5 KB Flash |
| **Tự động chạy khi bật nguồn** | `AGV_LINE_FOLLOW_AUTO_START` | [`Application/Inc/app_config.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_config.h) | Đếm ngược 3s rồi tự bám line trên sa bàn |
| **Tốc độ cơ sở bám line** | `AGV_LINE_FOLLOW_DEFAULT_RPM` | [`Application/Inc/app_config.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_config.h) | Vận tốc chạy thẳng (mặc định: 80 RPM) |
| **Ngưỡng quang học IR Min/Max** | `APP_LINE_CALIB_MIN_RAW`<br>`APP_LINE_CALIB_MAX_RAW` | [`Application/Inc/app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h) | Ngưỡng trắng/đen cho 5 mắt `[R2, R1, MID, L1, L2]` |
| **Đảo chiều Động cơ Phần mềm** | `cfg_left.inverted = true` | [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c#L69) | Đảo chiều quay động cơ không cần hàn lại dây |
| **Đảo chiều Encoder Phần mềm** | `cfg_left.inverted = true` | [`BSP/Src/bsp_encoder.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_encoder.c#L36) | Đảo dấu đếm xung encoder |
| **Đảo chiều Encoder Phần cứng** | `sConfig.IC1Polarity` | [`Core/Src/tim.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Core/Src/tim.c#L52) | Đảo dấu đếm qua phần cứng Timer 0% CPU |
| **Độ phân giải Encoder (CPR)** | `MOTOR_ENCODER_CPR` (1320) | [`Application/Src/app_motor_control.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Src/app_motor_control.c#L22) | Số xung 4X trên 1 vòng bánh xe |
| **Hệ số PI Motor & Feedforward**| `kp`, `ki`, `k_ff`, `deadzone` | [`Services/Inc/motor_pi.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Inc/motor_pi.h#L18) | Căn chỉnh bám vận tốc bánh xe |
| **Hệ số Line PID & Đánh lái** | `kp`, `ki`, `kd`, `out_max` | [`Application/Src/app_line_follow.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Src/app_line_follow.c#L25) | Căn chỉnh độ nhạy bẻ cua bám vạch |
| **Trọng số Vị trí Mắt IR** | `weights` | [`Services/Src/line_tracker.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Src/line_tracker.c#L82) | Trọng số sai số theo khoảng cách thực tế (1.25cm & 3.25cm): `[+1000, +385, 0, -385, -1000]` |
| **Tần số & Chu kỳ PWM** | `TIM4_PERIOD` (3599) | [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c#L44) | Tần số siêu âm 20 kHz, tự động co giãn duty |

---

## 2. Chi Tiết Các Nhóm Cấu Hình

### 2.1. Chế Độ Hoạt Động Của Hệ Thống (`Application/Inc/app_config.h`)

File này là trung tâm điều phối chế độ biên dịch và hành vi cấp cao của xe:

```c
/* 1: Chế độ Bàn test (kích hoạt CLI RTT, diag_motor, diag_ir) - Flash chiếm 59 KB
   0: Chế độ Vận hành Thực tế / Thi đấu (bóc tách toàn bộ code test) - Flash chiếm 42.5 KB */
#define AGV_ENABLE_MOTOR_TEST_MODE      1

/* 1: Tự động khởi động bám line sau khi cấp nguồn (rút cáp ST-Link chạy dưới sàn)
   0: Chờ lệnh RTT từ máy tính */
#define AGV_LINE_FOLLOW_AUTO_START      0

/* Tốc độ cơ sở khi tự hành bám line (RPM) */
#define AGV_LINE_FOLLOW_DEFAULT_RPM     80.0f

/* Thời gian đếm ngược an toàn (ms) trước khi xe bắt đầu lăn bánh (LED PC13 nháy nhanh 100ms) */
#define AGV_LINE_FOLLOW_ARM_DELAY_MS    3000
```

> [!TIP]
> **Quy trình chuẩn khi đi thi đấu hoặc vận hành thực tế**:
> 1. Đặt `AGV_ENABLE_MOTOR_TEST_MODE = 0` và `AGV_LINE_FOLLOW_AUTO_START = 1`.
> 2. Biên dịch `./build.sh` và nạp `./flash.sh`.
> 3. Rút cáp ST-Link, đặt xe ngay ngắn lên vạch xuất phát, bật công tắc pin. Sau 3 giây nháy đèn LED cảnh báo, xe sẽ tự động phóng đi bám line mượt mà.

---

### 2.2. Hiệu Chuẩn Ngưỡng Cảm Biến Quang Học IR (`Application/Inc/app_line_calib_defaults.h`)

File này chứa ngưỡng ADC thô (12-bit, $0 - 4095$) tương ứng với bề mặt nền trắng (Min) và vạch đen (Max) của 5 mắt cảm biến:

```c
/* Thứ tự mảng: [R2 (Ngoài cùng phải), R1 (Phải), MID (Tâm), L1 (Trái), L2 (Ngoài cùng trái)] */
#define APP_LINE_CALIB_MIN_RAW   { 1867, 3375, 3263, 2658, 2162 }
#define APP_LINE_CALIB_MAX_RAW   { 4017, 3974, 3966, 4003, 3946 }
```

- Các giá trị này được tự động cập nhật khi bạn chạy script `python scripts/calibrate_line.py --scan 10000`.
- Sau khi file này được cập nhật trên máy tính, bạn bắt buộc phải **biên dịch và nạp lại firmware** (`./build.sh && ./flash.sh`) để ghi giá trị mới vào bộ nhớ Flash của vi điều khiển.

---

### 2.3. Đảo Chiều Động Cơ & Đọc Xung Encoder

Do đặc thù cơ khí khung gầm đối xứng, động cơ hai bên thường được lắp quay mặt ra ngoài, dẫn đến một bên bị ngược chiều quay:

#### A. Đảo chiều Động cơ bằng Phần mềm (`BSP/Src/bsp_motor.c`)
```c
/* Cấu hình Động cơ Trái (Motor A) */
tb6612_cfg_t cfg_left = {
    .max_pwm = BSP_MOTOR_MAX_SPEED,
    .inverted = true   /* Đặt true để bánh trái quay TIẾN khi nhận lệnh dương */
};

/* Cấu hình Động cơ Phải (Motor B) */
tb6612_cfg_t cfg_right = {
    .max_pwm = BSP_MOTOR_MAX_SPEED,
    .inverted = false  /* Đặt false nếu bánh phải đã quay đúng chiều */
};
```

#### B. Đảo chiều Encoder Đếm Xung
- **Cách 1 (Phần cứng qua CubeMX - Khuyên dùng)**: Mở `agv_stm32.ioc` $\rightarrow$ TIM2 (hoặc TIM3) $\rightarrow$ Đổi `Input Polarity` từ `Rising Edge` sang **`Falling Edge`** (đã áp dụng cho TIM2 trong [`Core/Src/tim.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Core/Src/tim.c#L52)).
- **Cách 2 (Phần mềm qua BSP)**: Mở [`BSP/Src/bsp_encoder.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_encoder.c#L36) và đổi `.inverted = true`.

---

### 2.4. Điều Khiển Vận Tốc Bánh Xe (Motor PI & Feedforward)

Hệ thống điều khiển vận tốc 2 bánh xe vi sai hoạt động ở tần số $100\text{ Hz}$ ($10\text{ ms}$). Các thông số được định nghĩa tại [`Services/Inc/motor_pi.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Inc/motor_pi.h) và [`Application/Src/app_motor_control.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Src/app_motor_control.c):

| Tham số | Giá trị mặc định | Đơn vị | Ý nghĩa kỹ thuật |
| :--- | :---: | :---: | :--- |
| **`CPR`** | `1320` | Xung/vòng | Số xung thu được sau hộp giảm tốc khi bánh quay 1 vòng (chế độ 4X). |
| **`alpha`** | `0.85` | - | Hệ số lọc thông thấp số EMA cho vận tốc: $v_{filt} = \alpha v_{filt} + (1-\alpha) v_{raw}$. |
| **`kp`** | `5.04` | PWM/RPM | Hệ số khuếch đại tỉ lệ PI. Tăng để bám tốc độ nhanh; giảm nếu bánh xe gắt/rung. |
| **`ki`** | `112.3` | PWM/(RPM·s) | Hệ số tích phân. Triệt tiêu sai số xác lập khi xe tải nặng hoặc lên dốc. |
| **`k_ff`** | `3.705` | PWM/RPM | Hệ số bù tiến định ($1/K$). Đặt chính xác theo công thức $K_{ff} = 1000 / \text{RPM}_{\text{max}}$. |
| **`deadzone`**| `40.0` | PWM | Bù ma sát khởi động tĩnh (4% công suất). |
| **`max_accel`**| `600.0` | RPM/s | Giới hạn gia tốc dốc để tránh trượt bánh khi xuất phát hoặc sụt áp pin. |

#### Công thức tính $K_{ff}$ khi thay đổi điện áp nguồn $V_m$:
1. Chạy lệnh: `MOT A 1000 2500` qua RTT.
2. Đọc giá trị $\text{RPM}_{\text{max}}$ ổn định từ log RTT.
3. Tính toán:
   $$K_{ff} = \frac{1000}{\text{RPM}_{\text{max}}}$$

---

### 2.5. Điều Khiển Dò Line (Line PID & Tracker)

Các thông số bám vạch được cấu hình tại [`Application/Src/app_line_follow.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Src/app_line_follow.c) và [`Services/Src/line_tracker.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Src/line_tracker.c):

#### A. Trọng số hình học mảng cảm biến (`line_tracker.c`)
Tương ứng với khoảng cách thực tế từ mắt tâm (MID / Mắt 3 = 0 cm), mắt trong (R1, L1 = 1.25 cm), mắt ngoài (R2, L2 = 3.25 cm).
Tỉ lệ khoảng cách: $1.25 / 3.25 = 5 / 13 \approx 0.3846$ (chuẩn hóa về biên $\pm 1000$ tương ứng $\pm 385$):
```c
static const int32_t weights[LINE_TRACKER_NUM_SENSORS] = {1000, 385, 0, -385, -1000};
```
- Khi lệch sang phải ($R2, R1$): $e_{line} > 0$.
- Khi lệch sang trái ($L1, L2$): $e_{line} < 0$.
- Khi vạch nằm ngay mắt 2 hoặc 4 ($1.25\text{ cm}$): $e_{line} \approx \pm 385$.
- Khi vạch nằm ngay mắt 1 hoặc 5 ($3.25\text{ cm}$): $e_{line} \approx \pm 1000$.

#### B. Bộ điều khiển Line PID (`app_line_follow.c`)
```c
line_pid_cfg_t pid_cfg = {
    .kp = 0.08f,       /* Tỉ lệ đánh lái theo sai số tâm e_line */
    .ki = 0.005f,      /* Tích phân bù lệch tích lũy khi đi đường cong dài */
    .kd = 0.03f,       /* Vi phân chống vọt lố (damping) khi vào cua gắt */
    .out_max = 40.0f   /* Giới hạn chênh lệch tốc độ bẻ lái tối đa (+-40 RPM) */
};
```
- **Bán kính cua gắt**: Nếu xe bị văng khỏi vạch ở khúc cua gắt, hãy tăng `out_max` (ví dụ lên `50.0f` hoặc `60.0f`) và tăng nhẹ `kp`.
- **Hiện tượng xe lắc lư hình sin (Hunting)**: Nếu xe chạy thẳng nhưng thân xe bị lắc qua lắc lại liên tục, hãy giảm `kp` và tăng `kd`.
- **Thời gian an toàn mất vạch**:
  ```c
  #define LINE_LOST_TIMEOUT_MS  2000  /* Quá 2 giây mất vạch -> Ngắt motor khẩn cấp */
  ```

---

### 2.6. Ánh Xạ Phần Cứng PWM & Timer ARR (`BSP/Src/bsp_motor.c`)

- Timer 4 được CubeMX cấu hình ở tần số $20\text{ kHz}$ siêu âm:
  $$\text{Period} = 3599 \quad (\text{ARR} = 3600 \text{ xung})$$
- Tầng BSP tự động co giãn dải điều khiển chuẩn hóa $[0, 1000]$ sang thanh ghi so sánh phần cứng `CCR`:
  $$\text{pulse} = \frac{\text{duty} \times \text{ARR}}{1000} = \frac{\text{duty} \times 3599}{1000}$$
- **Ưu điểm**: Bạn có thể tùy ý thay đổi tần số Timer trong CubeMX (ví dụ đổi Period thành 1799 cho 40kHz), hàm ánh xạ tầng BSP sẽ tự động đọc `__HAL_TIM_GET_AUTORELOAD(&htim4)` mà không cần sửa code C.

