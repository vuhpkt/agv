# Đặc tả Kỹ thuật: Hệ thống Cảm biến IR & Điều khiển Bám Line AGV (SPEC-line-tracking)

## 1. Giả định Ban đầu (Assumptions)
1. **Phần cứng cảm biến**: Thanh 5 mắt hồng ngoại analog (IR reflectance sensor bar) kết nối vào 5 chân: `PA2` (Mắt 1 - Ngoài cùng trái), `PA3` (Mắt 2), `PA4` (Mắt 3 - Trung tâm), `PA5` (Mắt 4), `PB0` (Mắt 5 - Ngoài cùng phải).
2. **Đặc tuyến tín hiệu**: Tín hiệu đọc qua ADC1 12-bit (`0` đến `4095`), điện áp `0V` đến `3.3V`. Mắt cảm biến cho giá trị ADC cao khi gặp bề mặt sáng/trắng (phản xạ mạnh) và giá trị thấp khi gặp vạch đen (hấp thụ ánh sáng), hoặc ngược lại tùy loại module $\rightarrow$ hệ thống cần tính năng **Tự động Hiệu chuẩn (Auto-calibration min/max)** để tự thích ứng.
3. **Mô hình bám đường**: Đường line có độ rộng từ 1.5 cm đến 3.0 cm (chuẩn băng keo đen cách điện dán trên nền trắng).
4. **Cơ cấu chấp hành**: 2 bánh xe vi sai được điều khiển độc lập qua `velocity_service` (RPM vòng kín). Đầu ra của bộ điều khiển Line PID là lượng bù vận tốc góc $\Delta v$ (hoặc $\Delta \text{RPM}$):
   $$v_{left} = v_{base} + \Delta v, \quad v_{right} = v_{base} - \Delta v$$
   *(Khi $e > 0$ lệch phải $\rightarrow \Delta v > 0 \rightarrow v_{left} > v_{right}$ để bẻ phải; khi $e < 0$ lệch trái $\rightarrow \Delta v < 0 \rightarrow v_{left} < v_{right}$ để bẻ trái).*
5. **Nút giao / Giao lộ**: Nhận diện khi cả 5 mắt (hoặc tối thiểu 4 mắt) cùng vượt ngưỡng phát hiện vạch đồng thời (Cross-line / Vạch ngang).

---

## 2. Mục tiêu & Câu chuyện Người dùng (Objective & User Stories)

### 2.1. Mục tiêu
Xây dựng trọn gói mô-đun đọc cảm biến 5 mắt IR qua ADC1 DMA và dịch vụ điều khiển bám line tự hành kín (Line Tracking PID) trên STM32F103, giúp xe tự lái bám vạch mượt mà ở các dải tốc độ khác nhau, phát hiện chính xác nút giao, tuân thủ 100% kiến trúc 4 tầng và mô hình bộ nhớ tĩnh.

### 2.2. User Stories & Tiêu chí Nghiệm thu (Acceptance Criteria)

- **US-1: Đọc ADC DMA 0% CPU**
  - *Khi* hệ thống khởi động, ADC1 và DMA1 Channel 1 tự động quét vòng (Circular mode) liên tục 5 kênh.
  - *Thì* hàm `BSP_IR_GetRaw()` luôn trả về 5 giá trị ADC mới nhất tức thời ($O(1)$) mà không block CPU.
  
- **US-2: Hiệu chuẩn Trắng / Đen (Calibration)**
  - *Khi* kích hoạt chế độ hiệu chuẩn (quay xe qua lại trên vạch trong 2-3 giây), hệ thống tự ghi nhận giá trị cực tiểu ($min_i$) và cực đại ($max_i$) của từng mắt.
  - *Thì* mọi giá trị đọc sau đó được chuẩn hóa về dải $[0, 1000]$, trong đó $0$ là nền trắng và $1000$ là tâm vạch đen (hoặc ngược lại tùy cấu hình).

- **US-3: Ước lượng Vị trí Tâm Line ($e_{line}$)**
  - *Khi* xe nằm trên vạch line, thuật toán trọng số trung bình (Weighted Centroid) tính toán sai số độ lệch tâm $e_{line} \in [-1000, +1000]$:
    - $e_{line} = 0$: Vạch nằm chính giữa mắt trung tâm (PA4).
    - $e_{line} < 0$: Xe đang lệch phải (vạch nằm lệch về bên trái).
    - $e_{line} > 0$: Xe đang lệch trái (vạch nằm lệch về bên phải).
  - *Khi* cả 5 mắt bị mất vạch (Lost line), thuật toán ghi nhớ hướng lệch cuối cùng để hỗ trợ xe tự quay tìm lại vạch.

- **US-4: Điều khiển Bám Line (Line PID)**
  - *Khi* kích hoạt bám line ở tốc độ cơ sở $v_{base}$, bộ điều khiển PID tính toán lượng bẻ lái $\Delta v = K_p \cdot e + K_i \int e + K_d \frac{de}{dt}$.
  - *Thì* xe tự bám theo đường cong bán kính nhỏ mà không bị vọt lố hay lắc lư hình sin (hunting effect).

- **US-5: Phát hiện Vạch ngang Nút giao (Cross-line Detection)**
  - *Khi* xe đi qua ngã tư hoặc trạm dừng (tất cả các mắt đều bắt được vạch), cờ `is_crossline` được bật và kích hoạt sự kiện nút giao kèm khoảng cách Odometry hiện tại.

---

## 3. Ngăn xếp Công nghệ & Công cụ (Tech Stack & Commands)

- **Vi điều khiển**: STM32F103C8T6 (72MHz, 64KB Flash, 20KB RAM)
- **Chuẩn C**: C11 (GNU11 Extensions), `arm-none-eabi-gcc`
- **Build System**: CMake + Ninja
- **Kiểm thử**: Python 3 `unittest` mô phỏng thuật toán trọng số và Line PID

### Các lệnh thực thi (Commands)
```bash
# Biên dịch mã nguồn
cmake --build build/Debug

# Chạy kiểm thử tự động
python -m unittest discover tests

# Nạp firmware vào vi điều khiển
./flash.sh

# Mở kết nối log và tương tác CLI
./rtt.sh
nc localhost 9090
```

---

## 4. Cấu trúc Mô-đun trong Kiến trúc 4 Tầng

```
agv_stm32/
├── BSP/
│   ├── Inc/bsp_ir.h           → Giao diện phần cứng đọc ADC DMA & Calib
│   └── Src/bsp_ir.c           → Triển khai DMA buffer, HAL_ADCEx_Calibration
├── Services/
│   ├── Inc/line_tracker.h     → Thuật toán chuẩn hóa, Centroid error, Line PID
│   └── Src/line_tracker.c     → Triển khai Pure C (Zero-HAL, 100% static)
├── Application/
│   ├── Inc/app_line_follow.h         → Máy trạng thái điều hướng AGV (State Machine)
│   ├── Inc/app_line_calib_defaults.h → Mảng hằng số quang học đo thực tế
│   └── Src/app_line_follow.c         → Tích hợp bsp_ir + line_tracker + velocity_service
├── scripts/
│   ├── monitor_ir.py          → Giám sát trực quan mảng cảm biến IR qua RTT
│   └── calibrate_line.py      → Đo Min/Max trên bàn test & tự sinh C header
└── tests/
    ├── test_line_tracker.py             → Unit test thuật toán bám line
    ├── test_line_calib_defaults.py      → Unit test hằng số calib baseline
    ├── test_calibrate_line_script.py    → Unit test script đo dải động
    ├── test_app_line_follow_integration.py → Unit test tích hợp luồng chính
    └── test_line_follow_edge_cases.py   → Unit test ngã tư & timeout mất vạch
```

---

## 5. Thiết kế Giao diện Chi tiết (Data Contracts & Interfaces)

### 5.1. Tầng BSP: `bsp_ir.h`
```c
#ifndef BSP_IR_H
#define BSP_IR_H

#include <stdint.h>
#include <stdbool.h>

#define BSP_IR_NUM_SENSORS 5

void BSP_IR_Init(void);
void BSP_IR_GetRaw(uint16_t out_raw[BSP_IR_NUM_SENSORS]);

#endif /* BSP_IR_H */
```

### 5.2. Tầng Services: `line_tracker.h`
```c
#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

#define LINE_TRACKER_NUM_SENSORS 5

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_max;           /**< Giới hạn tối đa của lượng bẻ lái delta_rpm */
} line_pid_cfg_t;

typedef struct {
    uint16_t min_raw[LINE_TRACKER_NUM_SENSORS];  /**< Giá trị ngưỡng trắng/đen cực tiểu */
    uint16_t max_raw[LINE_TRACKER_NUM_SENSORS];  /**< Giá trị ngưỡng trắng/đen cực đại */
    uint16_t threshold[LINE_TRACKER_NUM_SENSORS];/**< Ngưỡng phân biệt 0/1 */
    bool is_white_line;                          /**< true nếu line trắng nền đen */
} line_calib_t;

typedef struct {
    line_calib_t calib;
    line_pid_cfg_t pid_cfg;
    float integral;
    float prev_error;
    int16_t last_valid_error;
    bool line_lost;
    bool is_crossline;
} line_tracker_t;

void line_tracker_init(line_tracker_t *tracker, const line_pid_cfg_t *pid_cfg);
void line_tracker_set_calib(line_tracker_t *tracker, const line_calib_t *calib);
void line_tracker_update_calib_minmax(line_tracker_t *tracker, const uint16_t raw[5]);

/**
 * @brief Chuẩn hóa giá trị 5 cảm biến về [0 .. 1000]
 */
void line_tracker_normalize(const line_tracker_t *tracker, const uint16_t raw[5], uint16_t out_norm[5]);

/**
 * @brief Tính sai số vị trí vạch [-1000 .. +1000] theo phương pháp trọng số
 */
int16_t line_tracker_compute_error(line_tracker_t *tracker, const uint16_t norm[5]);

/**
 * @brief Tính toán lượng bù vận tốc góc delta_rpm dựa trên Line PID
 */
float line_tracker_compute_pid(line_tracker_t *tracker, int16_t error, float dt_s);

#endif /* LINE_TRACKER_H */
```

---

## 6. Mô hình Toán học của Thuật toán (Mathematical Formulation)

### 6.1. Trọng số Vị trí (Weighted Centroid Formula)
Căn cứ theo khoảng cách hình học thực tế của thanh cảm biến ($1.25\text{ cm}$ cho mắt phụ cận và $3.25\text{ cm}$ cho mắt ngoài cùng từ tâm):
$$W = \begin{bmatrix} +1000 & +385 & 0 & -385 & -1000 \end{bmatrix}$$
Ánh xạ theo thứ tự 5 kênh ADC: `[CH0(R2), CH1(R1), CH2(MID), CH3(L1), CH4(L2)]`.

Sai số tâm vạch $e_{line} \in [-1000, +1000]$ được tính bằng mô men trọng số bậc 1 chia cho tổng cường độ phản xạ:

$$e_{line} = \frac{\sum_{i=0}^{4} W_i \times S_{norm, i}}{\sum_{i=0}^{4} S_{norm, i}}$$

- Nếu $\sum S_{norm, i} < S_{threshold}$ (cả 5 mắt đều không thấy vạch $\rightarrow$ mất vạch):
  - Giữ nguyên hướng lệch hợp lệ cuối cùng $e_{line} = \text{sign}(e_{prev}) \times 1000$.
  - Xe giảm tốc còn 60% tốc độ cơ sở để dò lại vạch.
  - Kích hoạt timer an toàn: ngắt khẩn cấp động cơ nếu mất vạch liên tục quá 2.0 giây (`LINE_LOST_TIMEOUT_MS = 2000`, `DIAG_IR_LOST_TIMEOUT_MS = 2000`).
- Nếu tối thiểu 4/5 mắt có $S_{norm, i} > 300$: Đánh dấu cờ `is_crossline = true` (phát hiện nút giao/vạch ngang).

### 6.2. Bộ điều khiển Line PID & Phân cực Vi sai
Để loại bỏ triệt để hiện tượng dao động lắc lư hình sin (hunting), thành phần tích phân được triệt tiêu hoàn toàn ($K_i = 0.0f$):
$$\Delta v = K_p \cdot e_{line} + K_d \frac{e_{line} - e_{prev}}{\Delta t}$$
*(Giá trị chuẩn hóa: $K_p = 0.15f, K_i = 0.0f, K_d = 0.035f$)*.

Giới hạn bẻ lái thích ứng theo tốc độ cơ sở để bánh trong có thể phanh dừng khi cua gắt:
$$\text{out\_max} = \max(60.0f, v_{base})$$

Tốc độ đặt từng bánh xe (Hồi tiếp âm ổn định):
$$\begin{cases} 
\omega_{left} = v_{base} + \Delta v \\ 
\omega_{right} = v_{base} - \Delta v 
\end{cases}$$
- Khi $e_{line} > 0$ (vạch bên phải): $\Delta v > 0 \implies \omega_{left} > \omega_{right} \implies$ Xe bẻ lái sang **PHẢI**.
- Khi $e_{line} < 0$ (vạch bên trái): $\Delta v < 0 \implies \omega_{left} < \omega_{right} \implies$ Xe bẻ lái sang **TRÁI**.

---

## 7. Ranh giới Kiến trúc & Quy chuẩn (Boundaries)

- **LUÔN LUÔN (Always):**
  - Tầng `BSP` (`bsp_ir.c`) chịu trách nhiệm kích hoạt `HAL_ADC_Start_DMA` và `HAL_ADCEx_Calibration_Start`.
  - Tầng `Services` (`line_tracker.c`) phải là **Pure C**, không gọi HAL, không phụ thuộc phần cứng, 100% dữ liệu truyền qua tham số hàm.
  - Viết Unit Test tự động cho hàm tính trọng số, hàm chuẩn hóa, và Line PID trước khi ghép nối hệ thống.
- **HỎI TRƯỚC (Ask first):**
  - Thay đổi cấu hình chân ADC hoặc thay đổi tần số vòng quét của DMA.
- **TUYỆT ĐỐI KHÔNG (Never):**
  - Không gọi `malloc()` trong quá trình chạy.
  - Không chèn `HAL_Delay()` vào bất kỳ hàm đọc hoặc tính toán nào.
  - Không để ngắt ADC DMA làm gián đoạn vòng lặp 10ms của xe.

---

## 8. Tiêu chí Thành công (Success Criteria)

1. **Unit Test**: 100% các test case về tính toán $e_{line}$ (ở giữa, lệch nhẹ trái/phải, lệch cực đại, mất line, vạch ngang ngã tư) vượt qua qua bài test Python.
2. **Biên dịch**: Firmware STM32 biên dịch sạch không warning, không error (`cmake --build build/Debug`).
3. **Hiệu năng thời gian thực**: Thời gian tính toán của hàm `line_tracker_compute_error()` và `line_tracker_compute_pid()` tốn **$< 20\ \mu\text{s}$** trên STM32 @ 72MHz.
4. **Không chiếm dụng CPU cho đọc ngoại vi**: DMA1 Channel 1 tự động refresh mảng cảm biến ở chế độ nền.

