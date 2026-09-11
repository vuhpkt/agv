# Hướng Dẫn Kiểm Tra & Khắc Phục Ngược Chiều Động Cơ & Encoder (TB6612 AGV)

Tài liệu này hướng dẫn quy trình kiểm tra thực tế chiều quay của Động cơ và Encoder trên xe tự hành AGV, cùng bảng ma trận chẩn đoán và các bước cấu hình lại **STM32CubeMX**, phần cứng hoặc phần mềm khi phát hiện bị ngược chiều.

---

## 1. Quy Ước Chiều Chuyển Động Chuẩn

Để các thuật toán điều khiển vận tốc (Motor PI), động học vi sai (Kinematics), định vị (Odometry) và bám line (Line Follow) hoạt động đồng bộ, hệ thống quy ước:

1. **Chiều Động Cơ**:
   - Lệnh PWM dương (`+PWM`, ví dụ `+400`): Bánh xe phải quay theo chiều làm xe **TIẾN TỚI**.
   - Lệnh PWM âm (`-PWM`, ví dụ `-400`): Bánh xe phải quay theo chiều làm xe **LÙI LẠI**.
2. **Chiều Encoder**:
   - Khi bánh xe quay TIẾN: Số xung Encoder đọc được phải **TĂNG DƯƠNG** (`Delta > 0`, `Total Ticks > 0`).
   - Khi bánh xe quay LÙI: Số xung Encoder đọc được phải **GIẢM ÂM** (`Delta < 0`, `Total Ticks < 0`).

---

## 2. Quy Trình Kiểm Tra Thực Tế Trên Bàn Test

> [!IMPORTANT]
> **An Toàn**: Hãy kê gầm xe hổng 2 bánh khỏi mặt bàn trước khi test để tránh xe bị giật hoặc chạy tự do khi động cơ quay.

### Cách 1: Sử Dụng Công Cụ Hỗ Trợ Tương Tác Python (Khuyên dùng)
1. Khởi động OpenOCD RTT server trong 1 cửa sổ terminal:
   ```bash
   ./rtt.sh
   ```
2. Chạy script chẩn đoán trên cửa sổ terminal thứ 2:
   ```bash
   python scripts/test_motor_direction.py
   ```
3. Menu sẽ hiển thị 4 lựa chọn:
   - Phím `1`: Quay Motor A (Trái) TIẾN (+400 PWM trong 1.5 giây).
   - Phím `2`: Quay Motor A (Trái) LÙI  (-400 PWM trong 1.5 giây).
   - Phím `3`: Quay Motor B (Phải) TIẾN (+400 PWM trong 1.5 giây).
   - Phím `4`: Quay Motor B (Phải) LÙI  (-400 PWM trong 1.5 giây).
4. Quan sát bánh xe và trả lời câu hỏi `(y/n)` trên màn hình. Script sẽ tự động đối chiếu và đưa ra kết luận chẩn đoán.

---

### Cách 2: Gửi Lệnh Trực Tiếp Qua RTT Terminal
Nếu bạn dùng `nc localhost 9090`, `telnet localhost 9090` hoặc J-Link RTT Viewer:
- Lệnh quay Motor A:
  - Tiến: `MOT A 400` (hoặc `STEP_TEST A 400 1500`)
  - Lùi:  `MOT A -400`
- Lệnh quay Motor B:
  - Tiến: `MOT B 400` (hoặc `STEP_TEST B 400 1500`)
  - Lùi:  `MOT B -400`

Hệ thống sẽ chạy động cơ trong 1.5s - 2s, sau đó in ra kết quả:
```text
DATA_END
[STEP_TEST] TEST_PASS | Peak: 75.0 RPM | Steady: 72.3 RPM | Ticks: 1350 | Samples: 200
[DIR_CHECK] PASS: Motor PWM (400) & Encoder Ticks (1350) MATCH (+Forward/+Reverse)
```
Nếu có dấu hiệu ngược chiều giữa PWM và Ticks, dòng `[DIR_CHECK]` sẽ chuyển sang màu đỏ và hiển thị cảnh báo `WARNING: MISMATCH!`.

---

## 3. Ma Trận Chẩn Đoán & Phương Án Xử Lý

Khi bạn gửi lệnh quay **TIẾN (`+PWM`)**, hãy đối chiếu kết quả quan sát với bảng sau:

| Kịch bản | Bánh xe vật lý | Ticks Encoder | Đánh giá | Nguyên nhân | Khắc phục đề xuất |
| :---: | :---: | :---: | :--- | :--- | :--- |
| **1** | Quay **TIẾN** | **Dương (+)** | **HOÀN HẢO** | Chiều đấu nối và timer đã đúng chuẩn 100%. | Không cần làm gì. |
| **2** | Quay **TIẾN** | **Âm (-)** | **Ngược Encoder** | Động cơ quay đúng hướng xe tiến, nhưng pha đọc xung Phase A / Phase B bị ngược. | Đổi `Input Polarity` trong CubeMX (TIM2/TIM3) HOẶC đảo 2 dây Phase A/B. |
| **3** | Quay **LÙI** | **Âm (-)** | **Ngược Động cơ** | Động cơ quay ngược làm xe đi lùi. Do xe đi lùi nên encoder đếm âm là hợp lý so với chuyển động lùi. | Đổi chân `AIN1`/`AIN2` (hoặc `BIN1`/`BIN2`) trong CubeMX HOẶC đảo 2 dây ra động cơ. |
| **4** | Quay **LÙI** | **Dương (+)** | **Ngược Cả Hai** | Động cơ quay lùi nhưng encoder lại đếm dương. | Đổi chiều động cơ VÀ đổi pha encoder. |

---

## 4. Hướng Dẫn Cấu Hình Lại Trên STM32CubeMX

Nếu bạn muốn cấu hình lại chuẩn ngay từ file `agv_stm32.ioc`:

### 4.1 Đảo chiều Động cơ trong STM32CubeMX
Khi Motor A hoặc Motor B quay ngược hướng:
1. Mở file `agv_stm32.ioc` bằng STM32CubeMX.
2. Tại mục **Pinout view**:
   - Với **Motor A (Trái)**: Hoán đổi nhãn (User Label):
     - `PB3`: Đổi nhãn từ `AIN1` thành `AIN2`
     - `PA15`: Đổi nhãn từ `AIN2` thành `AIN1`
   - Với **Motor B (Phải)**: Hoán đổi nhãn:
     - `PB5`: Đổi nhãn từ `BIN1` thành `BIN2`
     - `PB6`: Đổi nhãn từ `BIN2` thành `BIN1`
3. Nhấn **Generate Code** (`Ctrl + S` hoặc `Alt + K`).
4. Biên dịch lại: `cmake --build build/Debug`.

---

### 4.2 Đảo chiều đếm Encoder trong STM32CubeMX
Khi bánh xe quay tiến nhưng Timer đếm số âm:
1. Mở file `agv_stm32.ioc` bằng STM32CubeMX.
2. Tại cây danh mục ngoại vi bên trái:
   - Với Encoder Trái: Vào **Timers** -> chọn **TIM2**.
   - Với Encoder Phải: Vào **Timers** -> chọn **TIM3**.
3. Tại tab **Parameter Settings** ở khung bên dưới:
   - Tìm mục **Encoder Mode** -> **Input Polarity** (mặc định là `Rising Edge`).
   - Đổi giá trị của Channel 1 (hoặc Channel 2) từ `Rising Edge` sang **`Falling Edge`**.
4. Nhấn **Generate Code**.
5. Cơ chế phần cứng của Timer STM32 sẽ tự động đếm ngược dấu xung, chuyển xung âm thành xung dương mà **0% tốn CPU**.

---

## 5. Các Phương Án Xử Lý Nhanh Thay Thế (Không Cần Sửa CubeMX)

Nếu bạn không muốn mở lại STM32CubeMX, bạn hoàn toàn có thể xử lý theo 2 cách cực kỳ đơn giản sau:

### Phương Án A: Đảo dây vật lý (Hardware)
- **Đảo chiều động cơ**: Rút 2 dây ngõ ra cấp cho động cơ trên Domino của mạch TB6612 (`AO1` & `AO2` cho motor A, hoặc `BO1` & `BO2` cho motor B) và cắm tráo vị trí cho nhau.
- **Đảo chiều encoder**: Tráo đổi vị trí 2 dây tín hiệu Phase A và Phase B cắm vào STM32 (`PA0` tráo với `PA1` cho TIM2; hoặc `PA6` tráo với `PA7` cho TIM3).

### Phương Án B: Đổi cờ phần mềm (Firmware Config)
Trong mã nguồn driver tầng BSP đã tích hợp sẵn cờ logic `.inverted`:
- **Đảo chiều động cơ**: Mở [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c):
  ```c
  /* Đổi inverted từ false sang true cho motor tương ứng */
  tb6612_cfg_t cfg_left = {
      .max_pwm = BSP_MOTOR_MAX_SPEED,
      .inverted = true   /* Đổi thành true nếu Motor Trái bị ngược */
  };
  ```
- **Đảo chiều encoder**: Mở [`BSP/Src/bsp_encoder.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_encoder.c):
  ```c
  /* Đổi inverted từ false sang true cho encoder tương ứng */
  encoder_cfg_t cfg_left = {
      .inverted = true,  /* Đổi thành true nếu Encoder Trái bị ngược */
      .cpr = 1320
  };
  ```
