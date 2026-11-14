# Giao thức Lệnh CLI & Dữ liệu Đo xa (CLI & Telemetry Protocol)

Tài liệu này đặc tả toàn bộ tập lệnh điều khiển dòng lệnh (CLI), định dạng gói tin dữ liệu đo xa (Telemetry Streaming) và cách sử dụng các công cụ Python trên máy tính để giao tiếp với xe AGV STM32.

---

## 1. Kênh Giao tiếp Hỗ trợ

Xe tự hành AGV hỗ trợ 2 kênh truyền thông song song:

1. **SEGGER RTT (Real-Time Transfer qua mạch nạp ST-Link V2 / SWD)** - *Kênh chính*:
   - Tốc độ cực cao (> 100 KB/s), chiếm dụng thời gian CPU vi điều khiển cực thấp (< 1 microgiây mỗi lần ghi log).
   - Không cần dây nối thêm, không chiếm chân UART, chạy trực tiếp trên 2 dây nạp SWDIO (PA13) và SWCLK (PA14).
   - Kết nối máy tính qua cổng TCP `localhost:9090` do OpenOCD mở (chạy `./rtt.sh`).
2. **USART1 TTL DMA (Chân PA9 / PA10)** - *Kênh mở rộng*:
   - Tốc độ baud mặc định: `115200 bps`, 8 data bits, 1 stop bit, no parity.
   - Nhận/truyền hoàn toàn qua DMA và hàng đợi vòng `ring_buffer`, không gây trễ vòng lặp chính.
   - Bật/tắt trong `Application/Inc/app_config.h` qua định nghĩa `AGV_ENABLE_UART_TTL`.

---

## 2. Bảng Tra cứu Toàn bộ Tập lệnh CLI (Command Reference)

Tất cả các lệnh đều là chuỗi ký tự ASCII kết thúc bằng ký tự xuống dòng `\r\n` hoặc `\n`. Không phân biệt hoa/thường.

| Lệnh (Command) | Tham số | Ví dụ | Ý nghĩa & Phản hồi |
| :--- | :--- | :--- | :--- |
| **`PING`** | Không có | `PING` | Kiểm tra kết nối. MCU đảo trạng thái LED PC13 và phản hồi `PONG`. |
| **`SET_LED`** | `<0 \| 1>` | `SET_LED 1` | Bật (`1`) hoặc tắt (`0`) LED PC13 trên bo mạch. Phản hồi: `Ack:LED=1`. |
| **`SET_SPEED`** | `<left> <right>` | `SET_SPEED 500 -500`<br>`400 -400` | Điều khiển PWM trực tiếp vòng hở (Open-loop). Dải giá trị: `[-1000, 1000]`. Tự động ngắt vòng kín. Có thể gõ trực tiếp 2 số. |
| **`STOP`** | Không có | `STOP` | Thả trôi dừng cả 2 động cơ, dừng các bài test đang chạy. Phản hồi: `Ack:STOP`. |
| **`BRAKE`** | Không có | `BRAKE` | Kích hoạt phanh chủ động ngắn mạch (Short brake). Xe dừng lại tức thì. Phản hồi: `Ack:BRAKE`. |
| **`GET_ENC`** | Không có | `GET_ENC` | Đọc số xung tích lũy 32-bit của 2 encoder. Phản hồi: `ENC:L=<cnt>,R=<cnt>`. |
| **`RESET_ENC`** | Không có | `RESET_ENC` | Đặt lại bộ đếm xung của cả 2 encoder về 0. Phản hồi: `Ack:RESET_ENC`. |
| **`MOT`** / **`STEP_TEST`** | `[A\|B\|LEFT\|RIGHT\|0\|1] [pwm] [dur_ms]` | `MOT A 400`<br>`MOT B -500 1500`<br>`STEP_TEST A 1000 2500` | **Kiểm tra đáp ứng xung & Chẩn đoán chiều động cơ/encoder** (Module `Diagnostics/diag_motor`):<br>- Hỗ trợ ký hiệu `A` (Trái) và `B` (Phải).<br>- Tự động dừng an toàn sau `duration_ms` (mặc định 2000ms).<br>- Trả về kết quả: Peak RPM, Steady RPM, Total Ticks và dòng log tự động đối chiếu chiều `[DIR_CHECK]`. |
| **`RPM`** | `<left_rpm> [right_rpm]` | `RPM 80.0`<br>`RPM 100.0 -100.0` | **Điều khiển vận tốc kín (Closed-loop Motor PI)**: Đặt vận tốc mục tiêu theo đơn vị vật lý RPM. Nếu chỉ nhập 1 số, cả 2 bánh nhận cùng tốc độ. |
| **`PI_STATUS`** | Không có | `PI_STATUS` | In bảng trạng thái chi tiết của bộ điều khiển PI: Trạng thái (ACTIVE/IDLE), Target RPM, Actual RPM, PWM ngõ ra, bộ hệ số Kp/Ki/Kff. |
| **`PI_TUNE`** | `<kp> <ki> [kff]` | `PI_TUNE 5.0 100.0 3.5` | Cập nhật trực tiếp bộ hệ số PI/Feedforward theo thời gian thực mà không cần nạp lại firmware. |
| **`LINE_TEST`** | `[mode] [args...]` | `LINE_TEST`<br>`LINE_TEST SENSOR 3000`<br>`LINE_TEST CALIB 5000`<br>`LINE_TEST RUN 80 5000`<br>`LINE_TEST STOP` | **Kiểm tra độc lập mảng 5 mắt IR** (Module `Diagnostics/diag_ir`):<br>- `SENSOR`: Giám sát dải động 5 mắt ADC thô.<br>- `CALIB`: Tự động quét và cập nhật min/max quang học nền/vạch.<br>- `RUN` / `TRACK`: Chạy thử nghiệm bám line vòng kín trong khoảng thời gian xác định.<br>- `STOP`: Dừng bài test và dừng động cơ ngay lập tức. |
| **`LINE_START`** | `[base_rpm]` | `LINE_START`<br>`LINE_START 90.0` | Kích hoạt tự hành bám line vòng kín liên tục với tốc độ cơ sở `base_rpm` (mặc định 80 RPM). |
| **`LINE_STOP`** | Không có | `LINE_STOP` | Dừng tự hành bám line và phanh dừng động cơ. |

---

## 3. Định dạng Dữ liệu Đo xa & Chẩn đoán (Telemetry Streaming Protocol)

Khi thực hiện bài test đáp ứng bước nhảy (`MOT`, `STEP_TEST`) hoặc kiểm tra đa cấp tốc độ (`RPM`), firmware tự động kích hoạt chế độ stream dữ liệu chuẩn CSV:

```text
DATA_START
Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS
DATA:0,0.0000,0,0.0,0,0.00,0.000
DATA:10,0.0100,400,40.0,0,0.00,0.000
...
DATA:1500,1.5000,0,0.0,0,0.00,0.000
DATA_END
[STEP_TEST] TEST_PASS | Peak: 75.2 RPM | Steady: 72.8 RPM | Ticks: 1350 | Samples: 200
[DIR_CHECK] PASS: Motor PWM (400) & Encoder Ticks (1350) MATCH (+Forward/+Reverse)
```

### Ý nghĩa các thông tin chẩn đoán:
1. `DATA:Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS`: Dữ liệu lấy mẫu tần số 100Hz phục vụ vẽ đồ thị và nhận dạng hàm truyền.
2. `[STEP_TEST]`:
   - `Peak`: Vận tốc cực đại tức thời ghi nhận được (RPM).
   - `Steady`: Vận tốc xác lập trung bình trong 40% cuối bài test (RPM) $\rightarrow$ Dùng để xác định $\text{RPM}_{\text{max}}$ khi đổi điện áp nguồn.
   - `Ticks`: Tổng số xung Encoder 32-bit tích lũy trong suốt quá trình chạy.
3. `[DIR_CHECK]`:
   - `PASS`: Chiều quay của động cơ (dấu PWM) và chiều đếm xung Encoder cùng dấu (+ hoặc -).
   - `WARNING: MISMATCH!`: Ngược dấu giữa PWM và Ticks $\rightarrow$ Cảnh báo cần đảo pha encoder hoặc cấu hình lại CubeMX.

---

## 4. Danh mục Bộ Công cụ Python Hỗ trợ trên Máy tính

Dự án đi kèm bộ công cụ chuyên dụng trong thư mục `scripts/`:

### 4.1. Chẩn đoán Chiều Động cơ & Encoder (`scripts/test_motor_direction.py`)
Menu tương tác kiểm tra bánh xe và encoder cho từng động cơ, tự động đưa ra kết luận chẩn đoán:
```bash
python scripts/test_motor_direction.py
```
- Phím 1/2: Test Motor A (Trái) Tiến/Lùi.
- Phím 3/4: Test Motor B (Phải) Tiến/Lùi.

### 4.2. Hiệu chuẩn Quang học 5 Mắt IR Bàn Test (`scripts/calibrate_line.py`)
Lấy mẫu quang học nền trắng / vạch đen, kiểm tra dải động $\Delta \ge 100$, lưu cấu hình và cập nhật file header C:
```bash
# Quét dải động trong 5 giây và lưu cấu hình
python scripts/calibrate_line.py --scan 5000 --save

# Xem lại kết quả đã lưu trước đó
python scripts/calibrate_line.py --load
```

### 4.3. Nhận dạng Đáp ứng Bước nhảy Động cơ (`scripts/plot_step_response.py`)
Kích hoạt bài test bước nhảy, tính toán các tham số đặc trưng ($K, \tau, t_r, t_s$, hàm truyền $G(s)$) và vẽ đồ thị kỹ thuật:
```bash
python scripts/plot_step_response.py --rtt
```

### 4.4. Khôi phục Nhanh Lỗi Git Index trên Windows (`scripts/fix_git_index.bat`)
Tự động dọn dẹp file `.git/index` bị 0-byte do sập nguồn/treo tiến trình mà không làm mất code đang sửa:
```cmd
scripts\fix_git_index.bat
```

