# AGV Autonomous Mobile Robot - Dual-MCU Architecture

[![Architecture](https://img.shields.io/badge/Architecture-Dual--MCU%20(ESP32--S3%20%2B%20STM32)-orange.svg)]()
[![STM32](https://img.shields.io/badge/STM32-STM32F103C8T6%20%40%2072MHz-blue.svg)]()
[![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Xtensa%20LX7%20%40%20240MHz-red.svg)]()
[![IDF](https://img.shields.io/badge/ESP--IDF-v6.x-brightgreen.svg)]()
[![Tests](https://img.shields.io/badge/Tests-156%20passed-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

Hệ thống xe tự hành AGV hai bánh vi sai bám vạch kẻ sàn trong nhà kho thông minh và sa bàn mô phỏng công nghiệp.

Dự án sử dụng kiến trúc phân tầng 2 vi điều khiển:
- **ESP32-S3 (Freenove ESP32-S3 WROOM CAM)**: Đảm nhận kết nối không dây BLE, cầu nối lệnh UART, điều phối lộ trình và chuyển tiếp dữ liệu lên máy tính, kết nối Camera.
- **STM32F103C8T6**: Bộ điều khiển chấp hành thời gian thực, đọc mảng cảm biến quang học IR qua ADC DMA, tính toán tâm vạch, điều khiển vận tốc hai bánh bằng bộ điều khiển PI và theo dõi vạch bằng PID.

Hai vi điều khiển giao tiếp qua UART ở tốc độ 115200 bps bằng giao thức nhị phân có mã kiểm tra CRC16-CCITT.

---

## Kiến Trúc Tổng Quan

```
┌─────────────────────────────────────────────────────────────┐
│               GIAO DIỆN ĐIỀU KHIỂN & CHẨN ĐOÁN              │
│          (Python CLI / BLE GUI Dashboard trên máy tính)     │
│         Các tính năng: REPL CLI, Step Test, Auto-Tuner      │
└──────────────────────────────┬──────────────────────────────┘
                               │ Bluetooth Low Energy (BLE NUS MTU >= 256)
┌──────────────────────────────▼──────────────────────────────┐
│ [ESP32-S3] - ĐIỀU PHỐI VÀ KẾT NỐI KHÔNG DÂY                 │
│ - Kit: Freenove ESP32-S3 WROOM CAM (ESP32-S3R8 Octal PSRAM) │
│ - Môi trường: ESP-IDF v6.x (FreeRTOS, NimBLE NUS)           │
│ - Chân UART: GPIO42 (TX) & GPIO41 (RX), cách ly bus camera  │
│ - Cổng nạp và debug: Cổng USB Type-C "UART" onboard         │
│ - Dịch lệnh văn bản BLE sang gói tin nhị phân               │
│ - Thu thập và chuyển tiếp log từ STM32 lên máy tính         │
│ - Có phần cứng cho Camera OV2640 và thẻ nhớ       │
└──────────────────────────────┬──────────────────────────────┘
                               │ UART1 <-> USART1 (115200 8N1 DMA Circular)
                               │ Khung truyền nhị phân có CRC16-CCITT
                               │ ESP32 GPIO42 (TX) <-> STM32 PA10 (RX)
                               │ ESP32 GPIO41 (RX) <-> STM32 PA9 (TX)
┌──────────────────────────────▼──────────────────────────────┐
│ [STM32] - ĐIỀU KHIỂN CHUYỂN ĐỘNG THỜI GIAN THỰC             │
│ - Vi điều khiển: STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)      │
│ - Kiến trúc 4 tầng Clean Architecture                       │
│ - Đọc ADC1 DMA 5 cảm biến IR (chu kỳ 10ms - 100Hz)          │
│ - Thuật toán tính trọng số tâm vạch và Line PID             │
│ - Đọc encoder phần cứng 4X (TIM2, TIM3)                     │
│ - Vòng kín vận tốc Motor PI 100Hz kết hợp bù ma sát         │
│   feedforward và chống tràn tích phân anti-windup clamping  │
│ - Hàng đợi lộ trình tự hành: Tự nhận diện vạch ngang và     │
│   chuyển bước lộ trình                                      │
│ - Tự ngắt an toàn khi mất vạch quá 2.0 giây                 │
└─────────────────────────────────────────────────────────────┘
```

---

## Cấu Trúc Thư Mục

Tài liệu, đặc tả và kế hoạch được đặt tại thư mục gốc dùng chung cho cả dự án; mã nguồn và bài kiểm thử nằm trong từng thư mục module:

```
AGV/
├── docs/                             # Tài liệu kỹ thuật, đặc tả và quyết định kiến trúc
│   ├── specs/                        # Đặc tả kỹ thuật các tính năng
│   │   ├── SPEC-line-tracking.md     # Thuật toán bám vạch và mảng 5 cảm biến IR
│   │   ├── SPEC-crossline-exit-turn.md # Lướt qua vạch ngang và rẽ phải 3 cảm biến
│   │   ├── SPEC-agv-esp32-binary-gateway.md # Cổng giao tiếp nhị phân ESP32
│   │   └── SPEC-agv-ble-gui.md       # Giao diện đồ họa chẩn đoán và Auto-Tuner
│   ├── decisions/                    # Bản ghi quyết định kiến trúc (ADR)
│   │   ├── ADR-001-4-tier-clean-architecture.md
│   │   ├── ADR-002-zero-bloat-bench-diagnostics-and-hardware-scaling.md
│   │   ├── ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md
│   │   ├── ADR-004-esp32s3-freenove-cam-migration-and-pinout.md
│   │   └── ADR-005-line-tracking-dynamics-geometry-and-safety.md
│   ├── agv_system_overview_for_esp32.md # Tổng quan hệ thống và thông số cơ khí
│   ├── esp32_uart_protocol_guide.md  # Sơ đồ nối dây và cấu trúc gói tin UART
│   ├── architecture.md               # Kiến trúc phần mềm 4 tầng trên STM32
│   ├── hardware_and_pinout.md        # Sơ đồ chân MCU, Timer PWM 20kHz, ADC DMA
│   ├── control_theory.md             # Mô hình toán học, Motor PI, Line PID và Auto-Tuner
│   ├── configuration_guide.md        # Bảng tra cứu thông số cấu hình và căn chỉnh PID
│   ├── diagnostics_and_bench_testing.md # Hướng dẫn chẩn đoán và chạy thử trên bàn test
│   ├── motor_encoder_direction_guide.md # Hướng dẫn xác định và đảo chiều motor, encoder
│   ├── cli_and_telemetry.md          # Giao thức lệnh CLI và phân tích dữ liệu đo xa
│   └── troubleshooting_and_faq.md    # Hướng dẫn xử lý sự cố thường gặp
│
├── tasks/                            # Kế hoạch và tiến độ thực hiện
│   ├── plan.md                       # Kế hoạch tổng thể các giai đoạn
│   └── todo.md                       # Danh sách công việc chi tiết kèm tiêu chí nghiệm thu
│
├── agv_stm32/                        # Firmware điều khiển chuyển động (STM32F103)
│   ├── Application/                  # Tầng ứng dụng (app_main, app_comms, ...)
│   ├── BSP/                          # Tầng điều khiển phần cứng (UART DMA, PWM, ADC)
│   ├── Diagnostics/                  # Tự kiểm tra phần cứng (diag_motor, diag_ir)
│   ├── Drivers/                      # Driver IC TB6612FNG, CMSIS, STM32 HAL
│   ├── Services/                     # Thuật toán thuần C (comms_protocol, motor_pi, ...)
│   ├── Core/                         # Mã khởi tạo từ STM32CubeMX
│   ├── scripts/                      # Script nhận dạng động cơ và vẽ đồ thị
│   ├── tests/                        # Bộ kiểm thử tự động (120 test cases)
│   ├── CMakeLists.txt                # Cấu hình biên dịch (CMake + Ninja)
│   ├── build.sh / flash.sh           # Script biên dịch và nạp firmware qua ST-Link
│   └── README.md                     # Tài liệu chi tiết riêng cho module STM32
│
└── agv_esp32/                        # Firmware gateway BLE và công cụ điều khiển (ESP32-S3)
    ├── components/                   # Các component ESP-IDF
    │   ├── ble_nus/                  # Dịch vụ BLE Nordic UART Service (NimBLE)
    │   ├── button/                   # Xử lý nút bấm BOOT (GPIO0)
    │   ├── comms_protocol/           # Giao thức nhị phân thuần C dùng chung với STM32
    │   └── uart_service/             # Driver UART DMA và task xử lý dữ liệu
    ├── main/                         # Ứng dụng chính và bộ điều phối lệnh
    ├── CMakeLists.txt                # Cấu hình biên dịch ESP-IDF
    ├── sdkconfig / sdkconfig.defaults# Cấu hình ESP-IDF v6.x
    └── tools/                        # Giao diện đồ họa Desktop GUI và công cụ dòng lệnh CLI
```

---

## Danh Mục Tài Liệu Kỹ Thuật

| Nhóm tài liệu | Tài liệu | Nội dung chính |
|---|---|---|
| **Tổng quan và giao tiếp** | [agv_system_overview_for_esp32.md](docs/agv_system_overview_for_esp32.md) | Tổng quan hệ thống, kích thước cơ khí, phân định vai trò hai MCU |
| | [esp32_uart_protocol_guide.md](docs/esp32_uart_protocol_guide.md) | Sơ đồ nối dây UART, mã kiểm tra CRC16, cấu trúc gói tin nhị phân |
| **Đặc tả kỹ thuật** | [SPEC-agv-esp32-binary-gateway.md](docs/specs/SPEC-agv-esp32-binary-gateway.md) | Đặc tả 4 module: protocol, uart_service, ble_bridge, python client |
| | [SPEC-line-tracking.md](docs/specs/SPEC-line-tracking.md) | Đặc tả mảng 5 cảm biến IR ADC DMA, Weighted Centroid, Line PID |
| | [SPEC-crossline-exit-turn.md](docs/specs/SPEC-crossline-exit-turn.md) | Đặc tả lướt qua vạch ngang, rẽ phải 3 cảm biến và Line PID |
| | [SPEC-agv-ble-gui.md](docs/specs/SPEC-agv-ble-gui.md) | Đặc tả giao diện đồ họa BLE và bộ nhận dạng Auto-Tuner |
| **Quyết định kiến trúc** | [ADR-001: Clean Architecture](docs/decisions/ADR-001-4-tier-clean-architecture.md) | Chuẩn hóa kiến trúc 4 lớp, tách biệt HAL khỏi tầng Services |
| | [ADR-002: Zero-Bloat Diagnostics](docs/decisions/ADR-002-zero-bloat-bench-diagnostics-and-hardware-scaling.md) | Tách biệt mã test khỏi production, tối ưu bộ nhớ Flash và RAM |
| | [ADR-003: Binary Protocol & RTT Removal](docs/decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md) | Kiến trúc 2 MCU qua giao thức nhị phân UART |
| | [ADR-004: Freenove ESP32-S3 CAM Migration & Pinout](docs/decisions/ADR-004-esp32s3-freenove-cam-migration-and-pinout.md) | Quy hoạch chân UART1 GPIO 42/41, bảo lưu camera và PSRAM |
| | [ADR-005: Line Tracking Dynamics & Safety](docs/decisions/ADR-005-line-tracking-dynamics-geometry-and-safety.md) | Trọng số hình học cảm biến, triệt tiêu hunting và ngắt an toàn |
| **Điều khiển STM32** | [architecture.md](docs/architecture.md) | Phân tích 4 tầng phần mềm trên STM32, bộ lập lịch non-blocking |
| | [hardware_and_pinout.md](docs/hardware_and_pinout.md) | Sơ đồ chân STM32, TB6612, Encoder, 5 cảm biến IR, AFIO Remap |
| | [control_theory.md](docs/control_theory.md) | Hàm truyền động cơ, Motor PI, Line PID và thuật toán Auto-Tuner |
| | [configuration_guide.md](docs/configuration_guide.md) | Bảng tra cứu tham số PID, cấu hình chiều quay động cơ |
| | [motor_encoder_direction_guide.md](docs/motor_encoder_direction_guide.md) | Xác định và kiểm tra chiều quay bánh xe, chiều đếm xung encoder |
| | [diagnostics_and_bench_testing.md](docs/diagnostics_and_bench_testing.md) | Chạy thử nghiệm chẩn đoán động cơ và mảng cảm biến IR |
| | [troubleshooting_and_faq.md](docs/troubleshooting_and_faq.md) | Hướng dẫn chẩn đoán lỗi nạp, rung lắc line và xử lý sự cố |

---

## Giấy Phép

Dự án được phân phối theo giấy phép [MIT License](LICENSE).
