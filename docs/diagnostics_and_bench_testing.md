# Hướng Dẫn Kiểm Thử Bàn Test & Chẩn Đoán Phần Cứng (Diagnostics & Bench Testing)

Tài liệu này cung cấp hướng dẫn toàn diện về hệ thống chẩn đoán độc lập (Diagnostics Layer) và bộ công cụ Python trên máy tính (Host PC) phục vụ việc nghiệm thu từng cụm phần cứng (Động cơ, Encoder, Mảng IR) của xe tự hành AGV trước khi vận hành thực tế.

---

## 1. Triết Lý Thiết Kế: Zero-Bloat Bench Diagnostics (ADR-002)

Khi phát triển robot di động, các bài kiểm tra phần cứng (Step Response, đo chiều quay, hiệu chuẩn cảm biến quang học) thường đòi hỏi nhiều chuỗi văn bản log, thuật toán phân tích dải động và giao diện gỡ lỗi. 

Hệ thống AGV STM32 giải quyết bài toán này theo nguyên tắc:
1. **Chia tách trách nhiệm**:
   - **STM32 Firmware**: Chỉ thực hiện đo đạc chu kỳ thực tế (lấy mẫu $10\text{ ms}$ / $100\text{ Hz}$), xuất dữ liệu raw dạng compact qua SEGGER RTT (SWD).
   - **Host PC (Python Scripts)**: Chịu trách nhiệm phân tích dữ liệu, tính toán đặc tuyến quá độ, kiểm tra dải động $\Delta \ge 100$, hiển thị trực quan và tự động sinh mã nguồn C.
2. **Loại bỏ mã rác trong bản thương mại (Linker GC)**:
   - Khi `#define AGV_ENABLE_MOTOR_TEST_MODE 1` ([`Application/Inc/app_config.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_config.h)): Toàn bộ CLI và module Diagnostics hoạt động phục vụ gỡ lỗi trên bàn test.
   - Khi `#define AGV_ENABLE_MOTOR_TEST_MODE 0`: Vòng lặp chính cô lập hoàn toàn các module Diagnostics. Trình liên kết GCC với cờ `-Wl,--gc-sections` sẽ tự động bóc tách sạch sẽ toàn bộ code test và bộ thư viện chuỗi `sscanf/printf`, **giúp tiết kiệm ngay 16.5 KB Flash và 456 Bytes RAM** mà không còn 1 byte rác nào trong firmware chạy thực tế.

---

## 2. Danh Mục Các Công Cụ Python Hỗ Trợ

Toàn bộ các script nằm trong thư mục [`scripts/`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/) và giao tiếp với STM32 thông qua OpenOCD RTT Server (TCP port `9090`).

| Script | Mục đích kiểm tra | Lệnh thực thi | Đầu ra / Kết quả |
| :--- | :--- | :--- | :--- |
| [`test_motor_direction.py`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/test_motor_direction.py) | Kiểm tra chiều quay động cơ và chiều đếm encoder | `python scripts/test_motor_direction.py` | Ma trận chẩn đoán chiều quay & hướng dẫn khắc phục |
| [`monitor_ir.py`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/monitor_ir.py) | Giám sát trực quan 5 mắt IR thời gian thực | `python scripts/monitor_ir.py --duration 30000` | Thanh đo ASCII `█░`, giá trị Norm ($0-1000$), Raw ADC, $e_{line}$ |
| [`calibrate_line.py`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/calibrate_line.py) | Hiệu chuẩn Min (nền trắng) và Max (vạch đen) | `python scripts/calibrate_line.py --scan 10000` | Bảng PASS/FAIL $\Delta \ge 100$ & tự sinh [`app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h) |
| [`plot_step_response.py`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/plot_step_response.py) | Vẽ đặc tuyến đáp ứng bước nhảy động cơ | `python scripts/plot_step_response.py --rtt` | Đồ thị phân tích quá độ `step_response.png` |
| [`fix_git_index.bat`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/fix_git_index.bat) | Cứu hộ lỗi Git Index 0-byte trên Windows | `scripts\fix_git_index.bat` | Khôi phục index tức thời không mất code unstaged |

---

## 3. Chẩn Đoán Hệ Thống Truyền Động (Motor & Encoder Diagnostics)

### 3.1. Module Firmware: `Diagnostics/Src/diag_motor.c`
Module này phát xung điều khiển hở vòng (Open-loop PWM) trong một khoảng thời gian xác định, đồng thời ghi nhận số xung Encoder và tính vận tốc RPM tức thời mỗi $10\text{ ms}$:
- **Lệnh CLI qua RTT**:
  ```text
  MOT <A|B> <PWM> [duration_ms]
  ```
  *(Ví dụ: `MOT A 400 1500` = Kích Motor A quay với 40% PWM trong 1.5 giây)*
- **Định dạng dữ liệu trả về**:
  - Dòng dữ liệu thô: `DATA:STEP,<ms>,<target_pwm>,<actual_rpm>,<ticks>`
  - Dòng tổng kết: `[STEP_TEST] TEST_PASS | Peak: 75.0 RPM | Steady: 72.3 RPM | Ticks: 1350`
  - Dòng kiểm tra tương quan chiều: `[DIR_CHECK] PASS: Motor PWM (400) & Encoder Ticks (1350) MATCH (+Forward/+Reverse)`

### 3.2. Quy trình Chẩn đoán Chiều Quay Động cơ & Encoder
Khi lắp ráp khung gầm, động cơ hoặc encoder có thể bị ngược chiều so với quy ước của xe (Lệnh dương `+PWM` bánh xe phải quay **TIẾN** và Encoder phải đếm **TĂNG DƯƠNG**).

1. **Khởi động RTT Server**:
   ```bash
   ./rtt.sh
   ```
2. **Kê hổng bánh xe khỏi mặt đất** và chạy công cụ kiểm tra tương tác:
   ```bash
   python scripts/test_motor_direction.py
   ```
3. Menu sẽ hiển thị 4 bài test độc lập:
   - Phím `1`: Test Motor A (Trái) TIẾN (+400 PWM, 1.5s)
   - Phím `2`: Test Motor A (Trái) LÙI  (-400 PWM, 1.5s)
   - Phím `3`: Test Motor B (Phải) TIẾN (+400 PWM, 1.5s)
   - Phím `4`: Test Motor B (Phải) LÙI  (-400 PWM, 1.5s)
4. **Đối chiếu Ma trận Chẩn đoán**:
   - Nếu bánh xe quay lùi khi lệnh dương: Đổi `cfg.inverted = true` trong [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c).
   - Nếu bánh xe quay tiến nhưng Ticks đếm âm: Đổi `cfg.inverted = true` trong [`BSP/Src/bsp_encoder.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_encoder.c) hoặc đổi `IC1Polarity = Falling Edge` trong CubeMX [`Core/Src/tim.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Core/Src/tim.c).

### 3.3. Đo Tốc Độ Không Tải Cực Đại $\text{RPM}_{\text{max}}$ & Căn Chỉnh Feedforward $K_{ff}$
Khi thay đổi điện áp pin nguồn $V_m$ (ví dụ chuyển từ 5V sang pin 2S 7.4V):
1. Gửi lệnh kích xung 100% công suất:
   ```text
   MOT A 1000 2500
   ```
2. Đọc giá trị `Steady: <val> RPM` trong log `[STEP_TEST]`. Đó chính là $\text{RPM}_{\text{max}}$ thực tế ở mức điện áp đó.
3. Cập nhật lại hệ số Feedforward:
   $$K_{ff} = \frac{1000}{\text{RPM}_{\text{max}}}$$
   trong file [`Services/Inc/motor_pi.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Inc/motor_pi.h).

---

## 4. Chẩn Đoán Mảng Cảm Biến Dò Line (IR Array Diagnostics)

### 4.1. Module Firmware: `Diagnostics/Src/diag_ir.c`
Mảng cảm biến gồm 5 mắt quang học analog được đọc qua ADC1 Scan Mode + DMA1 Channel 1 ở chu kỳ $10\text{ ms}$. 

Thứ tự ánh xạ phần cứng được chuẩn hóa từ Phải sang Trái:
$$\text{CH0} = \mathbf{R2} \quad\longleftrightarrow\quad \text{CH1} = \mathbf{R1} \quad\longleftrightarrow\quad \text{CH2} = \mathbf{MID} \quad\longleftrightarrow\quad \text{CH3} = \mathbf{L1} \quad\longleftrightarrow\quad \text{CH4} = \mathbf{L2}$$

Module cung cấp 3 chế độ kiểm tra:
1. **`DIAG_IR_MODE_SENSOR` (Chế độ Giám sát)**: Đọc liên tục raw ADC 5 mắt, chuẩn hóa dải $0 - 1000$, tính sai số trọng tâm $e_{line}$, động cơ đứng yên.
2. **`DIAG_IR_MODE_CALIB` (Chế độ Hiệu chuẩn Quang học)**: Tự động cập nhật Min/Max trong RAM khi xe lia qua vạch.
3. **`DIAG_IR_MODE_TRACK` (Chế độ Chạy Thử Bám Line Độc Lập)**: Vừa đọc IR vừa chạy động cơ bám line kín ở tốc độ `base_rpm` với thời lượng giới hạn an toàn.

### 4.2. Giám Sát Trực Quan Thời Gian Thực (`monitor_ir.py`)
Khởi động giao diện màn hình trực quan với thanh bar-chart:
```bash
python scripts/monitor_ir.py --duration 30000
```
Giao diện hiển thị:
```text
==================================================
  AGV STM32 IR ARRAY DIAGNOSTICS - MODE: SENSOR
  Elapsed: 2450 ms | Line Detected: YES [ON TRACK]
  Centroid Error (e_line):    0 (Target: 0)
==================================================
  IR_R2 (Ch0) |░░░░░░░░░░░░░░░░░░░░░░░░░| Norm:   12 | Raw ADC: 1880
  IR_R1 (Ch1) |████████░░░░░░░░░░░░░░░░░| Norm:  340 | Raw ADC: 3580
  IR_MID(Ch2) |█████████████████████████| Norm:  990 | Raw ADC: 3960
  IR_L1 (Ch3) |████████░░░░░░░░░░░░░░░░░| Norm:  335 | Raw ADC: 3100
  IR_L2 (Ch4) |░░░░░░░░░░░░░░░░░░░░░░░░░| Norm:   15 | Raw ADC: 2180
==================================================
```
- **Ý nghĩa `Norm`**: Mức độ bắt vạch từ $0$ (nền trắng) đến $1000$ (tâm vạch đen).
- **Ý nghĩa `Centroid Error (e_line)`**: Sai số tâm vạch từ $-1000$ (vạch lệch cực đại về bên Trái) đến $+1000$ (vạch lệch cực đại về bên Phải). Khi vạch nằm chính giữa mắt MID, $e_{line} \approx 0$.

### 4.3. Quy Trình Hiệu Chuẩn Bàn Test (`calibrate_line.py`)
Khi chuyển xe sang sa bàn mới hoặc môi trường ánh sáng mới:
1. Đặt xe trên bàn gần máy tính, cắm cáp nạp ST-Link và chạy RTT Server: `./rtt.sh`.
2. Khởi chạy công cụ quét:
   ```bash
   python scripts/calibrate_line.py --scan 10000
   ```
3. Lia mảnh giấy trắng và băng dính đen qua lại dưới cả 5 mắt cảm biến trong 10 giây.
4. Script tự động:
   - Đánh giá dải động từng kênh: $\Delta_i = \text{Max}_i - \text{Min}_i \ge 100$.
   - Báo `FAIL` nếu có mắt bị tuột dây, chập mass (đọc giá trị $\approx 16$) hoặc LED phát bị cháy.
   - Báo `PASS` và tự động cập nhật file [`Application/Inc/app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h) và [`scripts/line_calib.json`](file:///d:/Workspace/Projects/AGV/agv_stm32/scripts/line_calib.json).
5. **Nạp firmware mới xuống vi điều khiển**:
   ```bash
   ./build.sh && ./flash.sh
   ```

---

## 5. Danh Mục Lệnh CLI Tra Cứu Nhanh (Cheat-Sheet)

Khi kết nối qua Netcat (`nc localhost 9090`) hoặc Telnet:

```text
# --- Lệnh Chẩn đoán Động cơ ---
MOT A 400 1500           # Test Motor A (Trái) 40% PWM trong 1.5s
MOT B 400 1500           # Test Motor B (Phải) 40% PWM trong 1.5s
MOT A 1000 2500          # Đo RPM max Motor A
STOP                     # Dừng toàn bộ động cơ ngay lập tức
BRAKE                    # Phanh ngắn mạch chủ động

# --- Lệnh Encoder & Vận tốc Vòng kín ---
GET_ENC                  # Đọc số xung encoder hiện tại: ENC:L=...,R=...
RESET_ENC                # Reset bộ đếm encoder về 0
RPM 80 80                # Đặt vận tốc vòng kín 80 RPM cho 2 bánh
PI_STATUS                # Xem chi tiết Target RPM, Actual RPM, Duty PWM
PI_TUNE 504 11230 370    # Đổi Kp=5.04, Ki=112.3, Kff=3.70 lúc đang chạy

# --- Lệnh Chẩn đoán Mảng Cảm biến IR ---
LINE_TEST SENSOR 15000   # Stream dữ liệu 5 mắt IR trong 15s (động cơ tắt)
LINE_TEST CALIB 10000    # Chế độ hiệu chuẩn tự động tìm min/max
LINE_TEST TRACK 60 5000  # Chạy thử nghiệm bám line kín 60 RPM trong 5s
LINE_TEST STOP           # Dừng test IR ngay lập tức

# --- Lệnh Vận hành Tự hành Tuyến Line ---
LINE_START 80            # Bắt đầu chạy bám line tự hành ở 80 RPM
LINE_STOP                # Dừng bám line an toàn
```

