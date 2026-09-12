# AGV STM32 - Differential Drive Robot Controller Firmware

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Tests](https://img.shields.io/badge/tests-94%20passed-brightgreen.svg)]()
[![MCU](https://img.shields.io/badge/MCU-STM32F103C8T6-blue.svg)]()
[![Architecture](https://img.shields.io/badge/architecture-4--tier%20Clean%20Arch-orange.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()

Firmware điều khiển xe tự hành AGV hai bánh vi sai (Differential Drive Mobile Robot) chạy trên vi điều khiển **STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)**. 

Dự án được thiết kế theo chuẩn **Clean Architecture / Inversion of Control**, tách biệt hoàn toàn thuật toán điều khiển khỏi phần cứng STM32 HAL, cấp phát bộ nhớ tĩnh 100% (Zero Dynamic Memory), tích hợp nhận dạng hàm truyền động cơ thực nghiệm, bộ điều khiển vận tốc PI vòng kín có bù ma sát tĩnh (Feedforward), và hệ thống bám line quang học 5 mắt IR tự hành thời gian thực.

---

## 📑 Mục lục Tài liệu Kỹ thuật Chi tiết

Dành cho kỹ sư mới tiếp cận dự án hoặc cần tra cứu sâu:

1. 🏛️ **[Kiến trúc Phần mềm Chi tiết](../docs/architecture.md)**: Phân tích sâu 4 tầng (Application $\rightarrow$ Services $\rightarrow$ BSP/Drivers $\rightarrow$ Core), nguyên lý Inversion of Control (IoC), mô hình bộ nhớ tĩnh, cơ chế lập lịch hợp tác non-blocking.
2. 🛠️ **[Hướng dẫn Cấu hình Hệ thống trong Code](../docs/configuration_guide.md)**: Bảng tra cứu toàn bộ macro, hằng số PID, đảo chiều động cơ, calib quang học IR, và thiết lập auto-start.
3. 🩺 **[Kiểm thử Bàn Test & Chẩn đoán Phần cứng](../docs/diagnostics_and_bench_testing.md)**: Hướng dẫn chi tiết các bài test `diag_motor`, `diag_ir` và bộ công cụ Python tương tác qua RTT/UART.
4. 🔌 **[Sơ đồ Phần cứng & Đấu nối Chân](../docs/hardware_and_pinout.md)**: Bảng tra cứu pinout đầy đủ, cấu hình Timer 20kHz PWM, Timer Encoder 4X, ADC1 Scan DMA 5 kênh IR (R2..L2), kỹ thuật AFIO Remap giải phóng chân JTAG.
5. 🔄 **[Hướng dẫn Kiểm tra & Đảo chiều Động cơ/Encoder](../docs/motor_encoder_direction_guide.md)**: Ma trận chẩn đoán chiều quay bánh xe và chiều đếm encoder, cách đảo chiều qua phần mềm hoặc CubeMX.
6. 📐 **[Lý thuyết Điều khiển & Nhận dạng Động cơ](../docs/control_theory.md)**: Mô hình hàm truyền thực nghiệm $G(s) = \frac{0.2699}{0.0449s + 1}$, cấu trúc PI Controller, bù Feedforward, chống bão hòa tích phân (Anti-windup), bộ lọc LPF EMA.
7. 🛣️ **[Đặc tả Kỹ thuật Bám Line & Cảm biến 5 Mắt IR](../docs/specs/SPEC-line-tracking.md)**: Thuật toán hiệu chuẩn min/max quang học, tính sai số trọng tâm Weighted Centroid, nhận diện ngã tư (Cross-line) và Line PID.
8. 📡 **[Giao thức Lệnh CLI & Dữ liệu Đo xa](../docs/cli_and_telemetry.md)**: Bảng tra cứu tập lệnh CLI qua RTT/UART, cấu trúc stream dữ liệu CSV đo xa, hướng dẫn các script Python tự động (`plot_step_response.py`, `test_multi_speed.py`).
9. 🆘 **[Sổ tay Xử lý Sự cố & Câu hỏi Thường gặp (FAQ)](../docs/troubleshooting_and_faq.md)**: Cẩm nang xử lý lỗi mạch nạp ST-Link/OpenOCD, đảo chiều motor/encoder, lệch mắt IR, lỗi Git Windows và checklist 5 bước trước khi đặt xe xuống sàn.
10. 📜 **[Bản ghi Quyết định Kiến trúc (ADRs)](../docs/decisions/)**:
   - [ADR-001: Chuẩn hóa Kiến trúc 4 Lớp & Tách biệt IC Driver](../docs/decisions/ADR-001-4-tier-clean-architecture.md)
   - [ADR-002: Kiến trúc Chẩn đoán Bàn Test Zero-Bloat & Chuẩn hóa Tỷ lệ Xung PWM Tầng BSP](../docs/decisions/ADR-002-zero-bloat-bench-diagnostics-and-hardware-scaling.md)
   - [ADR-003: Kiến Trúc 2 MCU, Giao Thức Nhị Phân UART & Gỡ Bỏ SEGGER RTT](../docs/decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md)

---

## 🚀 Khởi động Nhanh (Quick Start)

### 1. Yêu cầu Môi trường (Prerequisites)
- **Toolchain biên dịch**: `arm-none-eabi-gcc` (hỗ trợ C11)
- **Hệ thống build**: `cmake` (>= 3.22) và `ninja`
- **Mạch nạp & Debug**: `openocd` và mạch nạp ST-Link V2
- **Môi trường phân tích**: `python` (>= 3.8) với các thư viện `numpy`, `matplotlib`

### 2. Biên dịch Firmware
```bash
# Sử dụng script shell có sẵn
./build.sh

# Hoặc dùng CMake trực tiếp
cmake --build build/Debug
```

### 3. Nạp Firmware vào Vi điều khiển
Kết nối ST-Link V2 vào cổng SWD của STM32 (SWDIO: PA13, SWCLK: PA14, GND, 3V3):
```bash
./flash.sh
# Hoặc: cmake --build build/Debug -t flash
```

### 4. Kết nối Điều khiển & Xem Log Real-Time (SEGGER RTT)
```bash
# Bước 1: Khởi chạy OpenOCD RTT Server trên cổng 9090
./rtt.sh

# Bước 2: Mở một terminal khác kết nối qua Netcat hoặc Telnet
nc localhost 9090
# hoặc trên Windows PowerShell:
telnet localhost 9090
```

---

## 🗂️ Cấu trúc Thư mục Dự án

```
agv_stm32/
├── Application/               # Tầng Ứng dụng AGV cấp cao (Pure AGV Domain)
│   ├── Inc/                   # app_main.h, app_comms.h, app_motor_control.h, app_line_follow.h
│   └── Src/                   # app_main.c, app_comms.c, app_motor_control.c, app_line_follow.c
├── Diagnostics/               # Tầng Chẩn đoán & Tự kiểm tra Phần cứng (Hardware Self-Test)
│   ├── Inc/                   # diag_motor.h, diag_ir.h
│   └── Src/                   # diag_motor.c, diag_ir.c
├── BSP/                       # Tầng Board Support Package (Adapter phần cứng STM32)
│   ├── Inc/                   # bsp_board.h, bsp_motor.h, bsp_encoder.h, bsp_uart.h, bsp_ir.h
│   └── Src/                   # bsp_board.c, bsp_motor.c, bsp_encoder.c, bsp_uart.c, bsp_ir.c
├── Services/                  # Tầng Dịch vụ & Thuật toán thuần C (Zero-HAL, 100% Static)
│   ├── Inc/                   # line_tracker, velocity_service, comms_protocol, encoder, filter, kinematics, motor_pi, pid, ring_buffer
│   └── Src/                   # Các file .c tương ứng triển khai thuật toán
├── Drivers/                   # Driver phần cứng và ngoại vi
│   ├── tb6612/                # Driver điều khiển IC cầu H Toshiba TB6612FNG (tb6612.c, tb6612.h)
│   ├── SEGGER_RTT/            # Driver truyền nhận log tốc độ cao qua SWD của SEGGER
│   ├── CMSIS/                 # Thư viện lõi ARM Cortex-M3 Core
│   └── STM32F1xx_HAL_Driver/  # Thư viện STM32 HAL Drivers chính hãng từ ST
├── Core/                      # Khởi tạo phần cứng sinh từ CubeMX (main, tim, usart, dma, gpio, adc, it)
├── scripts/                   # Công cụ Python phân tích nhận dạng động cơ và vẽ đồ thị
│   ├── plot_step_response.py  # Thu thập và vẽ đặc tuyến bước nhảy động cơ
│   └── test_multi_speed.py    # Kiểm tra bám vận tốc đa cấp vòng kín
├── tests/                     # Bộ kiểm thử Unit Test tự động (110 bài test, chạy trong ~3.5s)
└── (Tài liệu chi tiết & ADRs đặt tại ../docs/)
```

---

## 🕹️ Tóm tắt Tập lệnh CLI (Cheat-Sheet)

Khi kết nối vào cổng RTT (`nc localhost 9090`), bạn có thể gõ trực tiếp các lệnh sau:

| Lệnh | Cú pháp | Mô tả |
| :--- | :--- | :--- |
| **PING** | `PING` | Kiểm tra kết nối, nhấp nháy LED PC13, trả về `PONG` |
| **Bật/Tắt LED** | `LED 1` hoặc `LED 0` | Điều khiển LED trạng thái PC13 |
| **Vận tốc Vòng hở**| `SPEED 600 -600` | Đặt PWM trực tiếp `[-1000, 1000]` cho 2 động cơ |
| **Dừng xe** | `STOP` | Thả trôi dừng động cơ, ngắt chế độ vòng kín |
| **Phanh khẩn cấp**| `BRAKE` | Phanh chủ động ngắn mạch motor |
| **Đọc Encoder** | `ENC` | Đọc số xung tích lũy 32-bit của 2 bánh |
| **Reset Encoder** | `RESET_ENC` | Đặt lại bộ đếm xung về 0 |
| **Vận tốc Vòng kín**| `VEL 120.0 120.0` | **Điều khiển PI vòng kín**: Đặt tốc độ bánh xe theo RPM |
| **Trạng thái PI** | `STATUS` | Xem chi tiết Target RPM, Actual RPM, PWM ngõ ra |
| **Căn chỉnh PI** | `TUNE 5.04 112.3 3.705` | Cập nhật bộ hệ số Kp, Ki, Kff ngay lúc xe đang chạy |
| **Test Bước nhảy** | `STEP 0 800 1500` | Kích hoạt chuỗi test đáp ứng bước nhảy động cơ (`diag_motor`) |
| **Test Dò Line IR**| `LINE_TEST [subcmd]` | **Kiểm tra độc lập mảng 5 mắt IR** (`diag_ir`):<br>`LINE_TEST`: Giám sát dải động 5 mắt ADC<br>`LINE_TEST CALIB 3000`: Quét min/max quang học<br>`LINE_TEST RUN 80 5000`: Chạy test bám line 80 RPM<br>`LINE_TEST STOP`: Dừng bài test và dừng xe |
| **Bám Line Tự Hành**| `LINE_START [rpm]` | Bắt đầu chạy bám line tự hành kín ở tốc độ chỉ định (mặc định: 80 RPM) |
| **Dừng Bám Line** | `LINE_STOP` | Dừng chế độ bám line, ngắt động cơ an toàn |

---

## 🎯 Quy trình Hiệu Chuẩn Bàn Test & Vận Hành Trên Sàn (Zero-Bloat)

Quy trình hiệu chuẩn cảm biến được tối ưu cho bàn làm việc gần laptop (cáp ST-Link ngắn) trước khi rút cáp đem xe ra chạy độc lập dưới sàn:

1. **Hiệu chuẩn trên bàn**: Kết nối xe với OpenOCD RTT (`./rtt.sh`), chạy công cụ Python:
   ```bash
   py scripts/calibrate_line.py --scan 5000
   ```
   Lia mảnh giấy trắng và băng keo đen qua 5 mắt cảm biến. Script tự động phân tích Min (nền trắng), Max (vạch đen), kiểm tra dải động quang học ($\Delta \ge 100$), in bảng kết quả trực quan và cập nhật trực tiếp vào `Application/Inc/app_line_calib_defaults.h`.

2. **Kiểm tra & Nạp code thủ công**:
   - Người dùng xem xét các giá trị Min/Max trên Terminal hoặc mở file `app_line_calib_defaults.h`.
   - Biên dịch và nạp firmware:
     ```bash
     ./build.sh
     ./flash.sh
     ```
   - Các thông số được nạp thẳng dạng hằng số tĩnh (`#define`) trong Flash $\rightarrow$ **0 byte RAM, 0 byte bộ nhớ cho parser chuỗi hay RTT trong firmware chính**.

3. **Chạy tự hành trên sàn**:
   - Rút cáp ST-Link, đặt xe xuống đường line sa bàn và bật công tắc nguồn.
   - Khi bật `#define AGV_LINE_FOLLOW_AUTO_START 1` trong `app_config.h`: Xe đếm ngược an toàn 3 giây (nháy LED PC13 nhanh 100ms để kịp đặt xe ngay ngắn) rồi tự động bám line mượt mà ở tần số 100Hz!
   - Tự động nhận diện ngã tư/vạch ngang (`CROSS_LINE`) và tự động ngắt motor khẩn cấp nếu mất vạch quá 2.0 giây (`LINE_LOST_TIMEOUT_MS`).

---

## 🧪 Kiểm thử Tự động (Automated Unit Testing)

Toàn bộ thuật toán điều khiển động cơ, mảng 5 mắt IR, bộ lọc LPF, bộ phân tích lệnh CLI, tính an toàn tràn số, các ranh giới kiến trúc, ngắt khẩn cấp mất vạch và script hiệu chuẩn được bảo vệ bởi 112 bài test tự động:

```bash
# Chạy toàn bộ 112 bài test tự động
py -3 -m unittest discover tests
```
*Kết quả mẫu:*
```
................................................................................................................
----------------------------------------------------------------------
Ran 112 tests in 1.359s

OK
```

---

## ⚡ Tài nguyên Vi điều khiển (Resource Footprint)

Firmware được thiết kế theo nguyên lý Zero-Bloat, tách biệt hoàn toàn giữa môi trường phát triển và sản phẩm thương mại:

| Chế độ cấu hình (`app_config.h`) | Flash Memory | RAM | Tiết kiệm |
| :--- | :--- | :--- | :--- |
| **Production Mode** (`AGV_ENABLE_MOTOR_TEST_MODE = 0`) | **42,576 B / 64 KB (64.97%)** | **7,728 B / 20 KB (37.73%)** | **Tiết kiệm 16,532 B Flash (-25.2%)** |
| **Bench Test Mode** (`AGV_ENABLE_MOTOR_TEST_MODE = 1`) | **59,108 B / 64 KB (90.19%)** | **8,184 B / 20 KB (39.96%)** | Đầy đủ RTT, CLI & Self-Test |

- **CPU Overhead**: Toàn bộ chu trình điều khiển vận tốc 2 bánh + bám line Line PID ở tần số 100 Hz chỉ chiếm **< 2.0%** năng lực xử lý của STM32 @ 72MHz.

