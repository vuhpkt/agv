# AGV Autonomous Mobile Robot - Dual-MCU Architecture

[![Architecture](https://img.shields.io/badge/Architecture-Dual--MCU%20(ESP32--S3%20%2B%20STM32)-orange.svg)]()
[![STM32](https://img.shields.io/badge/STM32-STM32F103C8T6%20%40%2072MHz-blue.svg)]()
[![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Xtensa%20LX7%20%40%20240MHz-red.svg)]()
[![IDF](https://img.shields.io/badge/ESP--IDF-v6.x-brightgreen.svg)]()
[![Tests](https://img.shields.io/badge/Tests-132%20passed-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

Hệ thống robot tự hành AGV hai bánh vi sai (Two-Wheel Differential Drive Mobile Robot) chuyên dụng cho bám vạch kẻ sàn (Line Tracking) trong nhà kho thông minh và sa bàn mô phỏng công nghiệp.

Dự án áp dụng mô hình **Kiến trúc Phân tầng 2 Vi điều khiển (2-Tier Dual-MCU Architecture)**, phân định ranh giới chặt chẽ giữa **Tầng Điều Phối Cấp Cao (High-Level Brain - Freenove ESP32-S3 CAM)** và **Tầng Chấp Hành Thời Gian Thực Cứng (Hard Real-Time Motion Controller - STM32F103)** qua đường truyền UART nhị phân có mã kiểm tra toàn vẹn CRC16-CCITT.

---

## 🏛️ Kiến Trúc Tổng Quan (System Architecture)

```
┌─────────────────────────────────────────────────────────────┐
│    NGƯỜI DÙNG / CLIENT TRÊN MÁY TÍNH & ĐIỆN THOẠI           │
│    (Python Bleak CLI Client / Mobile Web UI / App)          │
│    Các tính năng: REPL CLI, Step Test, Monitor IR, Calib    │
└──────────────────────────────┬──────────────────────────────┘
                               │ Bluetooth Low Energy (BLE NUS MTU >= 256)
┌──────────────────────────────▼──────────────────────────────┐
│ [ESP32-S3] - BỘ NÃO ĐIỀU PHỐI CẤP CAO (High-Level Master)   │
│ - Kit: Freenove ESP32-S3 WROOM CAM (ESP32-S3R8 Octal PSRAM) │
│ - Môi trường: ESP-IDF v6.x (v6.0.2, FreeRTOS, NimBLE NUS)   │
│ - Chân UART: GPIO42 (TX) & GPIO41 (RX) cách ly bus Camera   │
│ - Cổng nạp/debug: Cổng USB Type-C "UART" onboard            │
│ - Dịch lệnh text BLE sang khung truyền nhị phân chuẩn       │
│ - Thu thập và chuyển tiếp log debug từ STM32 lên máy tính   │
│ - Sẵn sàng tài nguyên cho Camera OV2640 & SD card tương lai │
└──────────────────────────────┬──────────────────────────────┘
                               │ UART1 <-> USART1 (115200 8N1 DMA Circular)
                               │ Khung truyền nhị phân có CRC16-CCITT
                               │ ESP32 GPIO42 (TX) <-> STM32 PA10 (RX)
                               │ ESP32 GPIO41 (RX) <-> STM32 PA9 (TX)
┌──────────────────────────────▼──────────────────────────────┐
│ [STM32] - BỘ ĐIỀU KHIỂN CHẤP HÀNH (Hard Real-Time Slave)    │
│ - Vi điều khiển: STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)      │
│ - Kiến trúc 4 tầng Clean Architecture, 100% Static Memory   │
│ - Đọc ADC1 DMA 5 mắt IR quang học (Chu kỳ 10ms - 100Hz)     │
│ - Thuật toán tính trọng số tâm vạch Weighted Centroid &     │
│   Line PID bám line mượt mà                                 │
│ - Đọc Hardware Timer Encoder 4X (TIM2, TIM3, CPR 1320)      │
│ - Vòng kín vận tốc động cơ Motor PI 100Hz + bù ma sát       │
│   Feedforward + chống tràn tích phân Anti-Windup Clamping   │
│ - Autonomous Route Queue: Tự đếm vạch ngang IR và tự nhảy   │
│   bước lộ trình mượt mà (không cần dừng chờ hỏi lại ESP32)  │
│ - Tự ngắt an toàn khi mất vạch liên tục (>1.0 giây)         │
└─────────────────────────────────────────────────────────────┘
```

---

## 🗂️ Cấu Trúc Thư Mục Dự Án (Project Structure)

Toàn bộ tài liệu, đặc tả và kế hoạch được tập trung ở cấp gốc để dùng chung cho cả hai nền tảng; các thư mục con chỉ chứa mã nguồn, công cụ kiểm thử và bản dựng:

```
AGV/
├── docs/                             # TOÀN BỘ TÀI LIỆU KỸ THUẬT & ĐẶC TẢ DỰ ÁN
│   ├── specs/                        # Đặc tả kỹ thuật các tính năng (Specifications)
│   │   ├── SPEC-line-tracking.md     # Đặc tả thuật toán bám line & mảng 5 mắt IR
│   │   └── SPEC-agv-esp32-binary-gateway.md # Đặc tả cổng Gateway nhị phân ESP32
│   ├── decisions/                    # Bản ghi quyết định kiến trúc (Architecture Decision Records)
│   │   ├── ADR-001-4-tier-clean-architecture.md
│   │   ├── ADR-002-zero-bloat-bench-diagnostics-and-hardware-scaling.md
│   │   └── ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md
│   ├── agv_system_overview_for_esp32.md # Bức tranh tổng thể & thông số vật lý AGV
│   ├── esp32_uart_protocol_guide.md  # Cẩm nang kết nối phần cứng & từ điển gói tin UART
│   ├── architecture.md               # Phân tích sâu kiến trúc phần mềm 4 tầng STM32
│   ├── hardware_and_pinout.md        # Sơ đồ chân MCU, Timer PWM 20kHz, ADC DMA
│   ├── control_theory.md             # Hàm truyền thực nghiệm G(s), Motor PI & Anti-windup
│   ├── configuration_guide.md        # Bảng tra cứu thông số cấu hình và căn chỉnh PID
│   ├── diagnostics_and_bench_testing.md # Hướng dẫn chẩn đoán và test bàn phần cứng
│   ├── motor_encoder_direction_guide.md # Cẩm nang xác định & đảo chiều motor/encoder
│   ├── cli_and_telemetry.md          # Giao thức lệnh CLI cũ & script phân tích đo xa
│   └── troubleshooting_and_faq.md    # Cẩm nang xử lý sự cố & câu hỏi thường gặp (FAQ)
│
├── tasks/                            # KẾ HOẠCH & TIẾN ĐỘ THỰC HIỆN
│   ├── plan.md                       # Bản kế hoạch tổng thể các giai đoạn phát triển
│   └── todo.md                       # Danh sách task chi tiết kèm tiêu chí nghiệm thu
│
├── agv_stm32/                        # FIRMWARE MOTION CONTROLLER (STM32F103)
│   ├── Application/                  # Tầng ứng dụng cấp cao (app_main, app_comms, ...)
│   ├── BSP/                          # Tầng adapter phần cứng STM32 (UART DMA, PWM, ADC)
│   ├── Diagnostics/                  # Tự kiểm tra phần cứng (diag_motor, diag_ir)
│   ├── Drivers/                      # Driver IC TB6612FNG, CMSIS, STM32 HAL
│   ├── Services/                     # Thuật toán Pure C (comms_protocol, motor_pi, ...)
│   ├── Core/                         # Mã khởi tạo vi điều khiển từ STM32CubeMX
│   ├── scripts/                      # Bộ script Python nhận dạng động cơ và vẽ đồ thị
│   ├── tests/                        # Bộ kiểm thử tự động (110 bài test, pass 100%)
│   ├── CMakeLists.txt                # Build configuration (CMake + Ninja)
│   ├── build.sh / flash.sh           # Script biên dịch và nạp firmware qua ST-Link
│   └── README.md                     # Tài liệu chi tiết riêng cho module STM32
│
└── agv_esp32/                        # FIRMWARE GATEWAY & COORDINATOR (ESP32)
    ├── components/                   # Các component ESP-IDF
    │   ├── ble_nus/                  # Dịch vụ BLE Nordic UART Service (NimBLE)
    │   ├── button/                   # Xử lý nút bấm BOOT (GPIO0)
    │   ├── comms_protocol/           # Giao thức nhị phân C thuần dùng chung với STM32
    │   └── uart_service/             # Driver UART2 DMA & task phân tích luồng byte
    ├── main/                         # Ứng dụng chính & bộ điều phối lệnh BLE/UART
    ├── CMakeLists.txt                # Build configuration (ESP-IDF CMake)
    ├── sdkconfig / sdkconfig.defaults# Cấu hình ESP-IDF v6.x
    └── tools/                        # Công cụ Python tương tác BLE không dây trên PC
```

---

## 📑 Mục Lục Tài Liệu Kỹ Thuật (Documentation Index)

| Nhóm Tài Liệu | Tài Liệu | Nội Dung Chính |
|---|---|---|
| **Tổng Quan & Gateway** | [agv_system_overview_for_esp32.md](docs/agv_system_overview_for_esp32.md) | Bức tranh tổng thể, kích thước cơ khí, phân định vai trò 2 MCU |
| | [esp32_uart_protocol_guide.md](docs/esp32_uart_protocol_guide.md) | Pinout nối dây UART, CRC16-CCITT, cấu trúc từng gói tin nhị phân |
| **Đặc Tả (Specs)** | [SPEC-agv-esp32-binary-gateway.md](docs/specs/SPEC-agv-esp32-binary-gateway.md) | Đặc tả 4 module: protocol, uart_service, ble_bridge, python client |
| | [SPEC-line-tracking.md](docs/specs/SPEC-line-tracking.md) | Đặc tả mảng 5 mắt IR ADC DMA, Weighted Centroid, Line PID |
| **Quyết Định (ADRs)** | [ADR-001: Clean Architecture](docs/decisions/ADR-001-4-tier-clean-architecture.md) | Chuẩn hóa kiến trúc 4 lớp, Zero-HAL Services, IoC Driver |
| | [ADR-002: Zero-Bloat Diagnostics](docs/decisions/ADR-002-zero-bloat-bench-diagnostics-and-hardware-scaling.md) | Tách biệt mã test khỏi production, tối ưu Flash/RAM |
| | [ADR-003: Binary Protocol & RTT Removal](docs/decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md) | Chuyển sang 2 MCU, giao thức nhị phân UART, bỏ SEGGER RTT |
| | [ADR-004: Freenove ESP32-S3 CAM Migration & Pinout](docs/decisions/ADR-004-esp32s3-freenove-cam-migration-and-pinout.md) | Chuyển sang Freenove S3 CAM, bảo lưu bus Camera, gán UART1 GPIO 42/41 |
| **STM32 Motion** | [architecture.md](docs/architecture.md) | Phân tích chi tiết 4 tầng STM32, Non-blocking scheduler |
| | [hardware_and_pinout.md](docs/hardware_and_pinout.md) | Bảng chân STM32, TB6612, Encoder, 5 mắt IR, AFIO Remap |
| | [control_theory.md](docs/control_theory.md) | Mô hình $G(s)$, Motor PI, bù Feedforward, Anti-windup clamping |
| | [configuration_guide.md](docs/configuration_guide.md) | Bảng tra cứu tham số PID, đảo chiều motor, auto-start |
| | [motor_encoder_direction_guide.md](docs/motor_encoder_direction_guide.md) | Ma trận chẩn đoán chiều quay bánh và chiều đếm xung encoder |
| | [diagnostics_and_bench_testing.md](docs/diagnostics_and_bench_testing.md) | Hướng dẫn chạy test bàn chẩn đoán motor và mảng IR |
| | [troubleshooting_and_faq.md](docs/troubleshooting_and_faq.md) | Xử lý lỗi nạp, lệch hướng, rung lắc line, mất vạch |

---

## 🚀 Hướng Dẫn Biên Dịch Nhanh (Quick Start)

### 1. Module STM32 (`agv_stm32`)
Yêu cầu: `arm-none-eabi-gcc`, `cmake`, `ninja`, `python 3.8+`
```bash
cd agv_stm32

# Biên dịch firmware
./build.sh
# Hoặc: cmake --build build/Debug

# Chạy toàn bộ 110 bài kiểm thử tự động
py -3 -m unittest discover tests

# Nạp firmware vào vi điều khiển qua ST-Link V2
./flash.sh
```

### 2. Module ESP32-S3 Gateway (`agv_esp32`)
Yêu cầu: ESP-IDF v6.x (v6.0.2)
```bash
cd agv_esp32

# Cấu hình target sang ESP32-S3 (lần đầu)
idf.py set-target esp32s3

# Biên dịch firmware
idf.py build

# Nạp firmware vào Freenove ESP32-S3 CAM qua cổng Type-C "UART" và mở log monitor
idf.py -p COMx flash monitor
```

### 3. Công Cụ Test Không Dây BLE Trên Máy Tính
```bash
pip install bleak
python agv_esp32/tools/agv_ble_controller.py
```
Các lệnh text hỗ trợ từ PC: `PING`, `STEP_TEST <motor> <pwm> <ms>`, `GET_ENC`, `RESET_ENC`, `ROUTE_TEST`, `STOP`.
