# Spec: AGV ESP32 Binary UART Gateway & Wireless BLE Bench-Test Client

## Phase 0: Capability Map

| Module ID | Responsibility | Depends On |
|---|---|---|
| `comms-protocol` | Mã nguồn C thuần giao thức nhị phân (`comms_protocol.c/.h`): CRC16-CCITT, byte-stream parser FSM, pack/unpack frame. | — |
| `uart-service` | Driver UART2 (GPIO16/17, 115200 8N1), FreeRTOS RX Task phân tích khung truyền nhị phân, bộ đệm gửi TX, callback dispatch frame cho ứng dụng. | `comms-protocol` |
| `ble-command-bridge` | Module `ble_cmd_dispatcher`: Biên dịch chuỗi text từ BLE NUS sang binary frame cho STM32; format phản hồi từ STM32 (ACK, PONG, LOG, STATUS, ENC) thành text gửi lại BLE NUS; hỗ trợ MTU trao đổi dữ liệu tốc độ cao. | `uart-service`, `ble_nus` |
| `tools-ble-client` | Bộ công cụ Python tương tác trên PC (`agv_esp32/tools/agv_ble_controller.py`) dùng thư viện `bleak`: kết nối BLE, giao diện REPL, thu thập và hiển thị đồ thị Step Response, vẽ thanh đo 5 mắt IR trực quan (`monitor_ir`), và quét hiệu chuẩn quang học bàn test (`calibrate_line`). | `ble-command-bridge` |

**Build order:** `comms-protocol` → `uart-service` → `ble-command-bridge` → `tools-ble-client`

---

## 1. Objective (Mục Tiêu)

Xây dựng tầng firmware truyền thông nhị phân trên **ESP32 (ESP-IDF v6.x / v6.0.2)** để kết nối đồng bộ với vi điều khiển chấp hành **STM32F103C8T6** theo chuẩn giao thức nhị phân `comms_protocol` (ADR-003), đồng thời cung cấp giao diện điều khiển và chẩn đoán toàn diện qua **Bluetooth Low Energy (NimBLE Nordic UART Service - NUS)**.

Song song đó, cung cấp bộ công cụ Python trên máy tính (`agv_esp32/tools/agv_ble_controller.py`) cho phép kỹ sư vận hành kết nối BLE không dây, thực hiện toàn bộ các bài test chẩn đoán phần cứng và xem kết quả trực quan trên PC thay thế hoàn toàn cáp nạp ST-Link / SEGGER RTT:
- **Kiểm tra kết nối & Đọc cảm biến**: Ping, Đọc/Reset Encoder 2 bánh.
- **Chẩn đoán đáp ứng bước động cơ (`plot_step`)**: Kích xung PWM, thu thập tốc độ động cơ theo thời gian thực và vẽ đồ thị đặc tuyến bước nhảy.
- **Giám sát thời gian thực mảng 5 mắt IR (`monitor_ir`)**: Hiển thị đồ họa thanh bar trực quan mức độ phản xạ ADC của từng mắt và vị trí sai số tâm vạch $e_{line}$.
- **Hiệu chuẩn quang học bàn test (`calibrate_line`)**: Quét min/max khi lia giấy trắng/vạch đen, kiểm tra dải động quang học $\Delta \ge 100$, và tự động xuất file cấu hình `app_line_calib_defaults.h`.
- **Điều khiển lộ trình & Dừng khẩn cấp**: Nạp chuỗi ngã rẽ, chạy tự hành và phanh dừng tức thời.

### User Stories & Acceptance Criteria

1. **US-1: Kiểm tra kết nối 2 chiều (Ping/Pong)**
   - *Khi* người dùng gửi lệnh `PING` qua BLE (hoặc ấn nút cứng BOOT trên ESP32).
   - *Thì* ESP32 đóng gói `AGV_MSG_PING` (ID `0x01`) gửi qua UART2 tới STM32; STM32 nháy LED PC13 và trả lời `AGV_MSG_PONG` (ID `0x81`); ESP32 gửi thông báo `[RESP] PONG (STM32 Alive)\r\n` về client BLE.
2. **US-2: Chạy thử đáp ứng bước động cơ (Step Test) & Vẽ đồ thị qua BLE**
   - *Khi* người dùng nhập `STEP_TEST <motor> <pwm> <ms>` hoặc lệnh client `step <motor> <pwm> <ms>`.
   - *Thì* ESP32 đóng gói `AGV_MSG_DIAG_STEP_TEST` (ID `0x20`) gửi sang STM32.
   - *Thì* STM32 chạy motor và stream dữ liệu `DATA:<ms>,<time_s>,<pwm>,<pct>,<ticks>,<rpm>,<rads>` qua log nhị phân; ESP32 chuyển tiếp toàn bộ dòng dữ liệu qua BLE NUS; Python client đón nhận, vẽ thanh tiến trình và lưu/vẽ đồ thị đáp ứng bước nhảy.
3. **US-3: Giám sát mảng 5 mắt IR thời gian thực qua BLE (`monitor_ir`)**
   - *Khi* người dùng nhập lệnh `MONITOR_IR [duration_ms]` (hoặc `monitor_ir`).
   - *Thì* ESP32 đóng gói `AGV_MSG_DIAG_LINE_TEST` (ID `0x26`, `mode = 0` SENSOR, `rpm = 0`) gửi sang STM32.
   - *Thì* STM32 đọc ADC DMA và phát dòng dữ liệu `DATA:LINE,<ms>,<raw0..4>,<norm0..4>,<err>,<line_seen>` ở tần số 20Hz; ESP32 bắn qua BLE; Python client render trực tiếp thanh đo đồ họa Unicode `|██████░░░░|` cho cả 5 mắt cảm biến, hiển thị trạng thái phát hiện vạch và sai số tâm line theo thời gian thực.
4. **US-4: Hiệu chuẩn quang học IR bàn test qua BLE (`calibrate_line`)**
   - *Khi* người dùng nhập lệnh `CALIBRATE_LINE [duration_ms]` (hoặc `calibrate`).
   - *Thì* ESP32 đóng gói `AGV_MSG_DIAG_LINE_TEST` (ID `0x26`, `mode = 1` CALIB).
   - *Thì* người dùng lia mẫu giấy trắng và băng keo đen qua 5 mắt; Python client tích lũy giá trị cực tiểu ($min_i$) và cực đại ($max_i$), kiểm tra dải động $\Delta \ge 100$, in bảng tổng kết và tự động sinh file header C `app_line_calib_defaults.h` và file `line_calib.json`.
5. **US-5: Đọc và Reset xung Encoder**
   - *Khi* người dùng gửi `GET_ENC`, ESP32 gửi `AGV_MSG_DIAG_GET_ENC` (ID `0x23`), đón nhận frame `AGV_MSG_DIAG_ENC_DATA` (ID `0xA3`) và hiển thị: `[ENCODER] Left: <ticks> | Right: <ticks>\r\n`.
   - *Khi* người dùng gửi `RESET_ENC`, ESP32 gửi `AGV_MSG_DIAG_RESET_ENC` (ID `0x24`) và trả về `[RESP] ENCODER RESET OK\r\n`.
6. **US-6: Nạp và kích hoạt lộ trình tự hành (Route Mission)**
   - *Khi* người dùng gửi `ROUTE_LOAD <rpm> <actions...>` (ví dụ: `ROUTE_LOAD 80.0 F,R,F,S`), ESP32 đóng gói `AGV_MSG_ROUTE_LOAD` (ID `0x10`).
   - *Khi* người dùng gửi `ROUTE_START`, ESP32 phát `AGV_MSG_ROUTE_START` (ID `0x11`).
   - *Khi* xe chạy bám line, các gói `AGV_MSG_MISSION_STATUS` (ID `0x90`) được ESP32 format và bắn liên tục về BLE: `[MISSION] State: RUNNING, Step: 1\r\n` ... `[MISSION] State: COMPLETED, Step: 4\r\n`.
7. **US-7: Dừng phanh khẩn cấp (Emergency Stop)**
   - *Khi* người dùng gửi `STOP` hoặc `EMG`, ESP32 phát ngay lập tức `AGV_MSG_EMERGENCY_STOP` (ID `0x02`), STM32 phanh cứng động cơ và hủy bài test.
8. **US-8: Kênh log đo xa không dây (Wireless Log Relay)**
   - *Khi* STM32 phát sinh bất kỳ log debug nào qua `AGV_LOG(...)`, frame `AGV_MSG_LOG` (ID `0x99`) được gửi sang ESP32 qua UART2; ESP32 bọc vào tiền tố `[STM32 LOG]` (hoặc truyền raw dòng `DATA:...` đối với telemetry) và notify ngay về BLE NUS.

---

## 2. Tech Stack (Công Nghệ & Môi Trường)

- **Vi điều khiển Gateway**: ESP32 (Dual-Core Xtensa LX6 @ 240MHz, 4MB Flash, 520KB SRAM).
- **Firmware Framework**: **ESP-IDF v6.x (`v6.0.2`)** (C11, FreeRTOS, NimBLE stack).
- **Giao tiếp phần cứng**:
  - **UART2**: TX = `GPIO17`, RX = `GPIO16`, Baudrate = `115200 bps`, `8N1`, No flow control.
  - **BOOT Button**: `GPIO0` (Active-LOW, pull-up nội, debounce phần mềm 50ms).
  - **BLE Stack**: NimBLE với profile Nordic UART Service (NUS):
    - Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
    - RX Characteristic (Write without response / Write): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
    - TX Characteristic (Notify): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
    - Device Name: `AGV_ESP32`
    - **MTU Negotiation**: Hỗ trợ mở rộng MTU lên tới 256 bytes để truyền tải trọn vẹn các dòng telemetry `DATA:LINE,...` mà không bị ngắt vụn gói.
- **PC Test Client**:
  - Python 3.10+ (chạy trên Windows/Linux/macOS).
  - Thư viện: `bleak >= 0.21.0`, `asyncio`, `matplotlib` (tùy chọn vẽ đồ thị bước nhảy).

---

## 3. Commands (Lệnh Thực Thi)

### 3.1 Firmware ESP-IDF (Windows PowerShell / ESP-IDF CMD)
```powershell
cd agv_esp32

# Cấu hình target (nếu thiết lập mới)
idf.py set-target esp32

# Biên dịch firmware
idf.py build

# Nạp firmware vào bo mạch ESP32
idf.py -p COMx flash

# Mở Serial Monitor để quan sát log UART gốc của ESP32
idf.py -p COMx monitor

# Tổ hợp nạp và mở monitor một bước
idf.py -p COMx flash monitor
```

### 3.2 Python Test Client (PC)
```powershell
# Cài đặt thư viện phụ thuộc
pip install bleak matplotlib

# 1. Chạy console tương tác đầy đủ tính năng:
python agv_esp32/tools/agv_ble_controller.py

# 2. Chạy trực tiếp chế độ Giám sát 5 mắt IR qua BLE:
python agv_esp32/tools/agv_ble_controller.py --monitor-ir 15000

# 3. Chạy trực tiếp chế độ Hiệu chuẩn bàn test qua BLE và xuất file header:
python agv_esp32/tools/agv_ble_controller.py --calibrate 10000

# 4. Chạy trực tiếp bài test đáp ứng bước và vẽ đồ thị:
python agv_esp32/tools/agv_ble_controller.py --step L 600 1500 --plot
```

---

## 4. Project Structure (Cấu Trúc Thư Mục)

```
AGV/
├── README.md                         # Tổng quan dự án, kiến trúc 2 MCU & index docs
├── docs/                             # TOÀN BỘ TÀI LIỆU DÙNG CHUNG
│   ├── specs/
│   │   ├── SPEC-agv-esp32-binary-gateway.md # Đặc tả kỹ thuật này
│   │   └── SPEC-line-tracking.md
│   ├── decisions/                    # ADR-001, ADR-002, ADR-003
│   └── ...                           # Tài liệu phần cứng, lý thuyết điều khiển, pinout
├── tasks/                            # plan.md & todo.md
│
├── agv_stm32/                        # TẦNG CHẤP HÀNH THỜI GIAN THỰC (STM32F103)
│
└── agv_esp32/                        # TẦNG GATEWAY KHÔNG DÂY & ĐIỀU PHỐI (ESP32)
    ├── CMakeLists.txt                # Cấu hình build gốc của ESP-IDF project
    ├── sdkconfig / sdkconfig.defaults# Cấu hình ESP-IDF v6.x (kích hoạt NimBLE, UART buffer)
    ├── components/
    │   ├── comms_protocol/           # [Component 1] Mã C thuần giao thức nhị phân
    │   │   ├── CMakeLists.txt
    │   │   ├── comms_protocol.c      # CRC16, agv_parser_feed_byte, agv_pack_*
    │   │   └── include/
    │   │       └── comms_protocol.h  # Struct packed & enum đồng bộ với STM32
    │   ├── uart_service/             # [Component 2] Driver UART2 & FreeRTOS parser task
    │   │   ├── CMakeLists.txt
    │   │   ├── uart_service.c        # Khởi tạo UART2, task feed byte vào parser
    │   │   └── include/
    │   │       └── uart_service.h    # API gửi frame nhị phân & callback nhận frame
    │   ├── ble_nus/                  # [Component 3] NimBLE Nordic UART Service
    │   │   ├── CMakeLists.txt
    │   │   ├── ble_nus.c             # GAP advertising, GATT table, notify callback, MTU config
    │   │   └── include/
    │   │       └── ble_nus.h         # ble_nus_send_str(), rx_callback, is_connected
    │   └── button/                   # [Component 4] Nút bấm BOOT (GPIO0)
    │       ├── CMakeLists.txt
    │       ├── button.c
    │       └── include/
    │           └── button.h
    ├── main/                         # Ứng dụng chính & Cầu nối BLE <-> UART
    │   ├── CMakeLists.txt
    │   ├── main.c                   # Khởi tạo nvs, uart_service, ble_nus, button
    │   ├── ble_cmd_dispatcher.h     # Interface bộ phân tích lệnh BLE text
    │   └── ble_cmd_dispatcher.c     # Parse text, pack frame xuống UART & format response
    └── tools/                        # Bộ công cụ Python PC chẩn đoán & điều khiển
        └── agv_ble_controller.py     # REPL CLI, Step Response Plotter, IR Monitor & Calib tool
```

---

## 5. Interface Contracts & Protocol Specification

### 5.1 Giao Tiếp Khung Truyền Nhị Phân UART (ESP32 <-> STM32)

Khung truyền chuẩn theo `comms_protocol`:
```
+--------+--------+--------+------------+-------------------------+------------+------------+
| SOF1   | SOF2   | MSG_ID | LENGTH     | PAYLOAD (0..128 bytes)  | CRC16_H    | CRC16_L    |
| (0xAA) | (0x55) | (1 B)  | (1 B: 0..N)| (N Bytes)               | (1 B)      | (1 B)      |
+--------+--------+--------+------------+-------------------------+------------+------------+
```

### 5.2 Bảng Lệnh BLE Cấp Cao & Các Chế Độ Chẩn Đoán

| Lệnh Text BLE | Tham Số | Gói Nhị Phân UART | Ý Nghĩa / Luồng Dữ Liệu Phản Hồi |
|---|---|---|---|
| `PING` | Không | `AGV_MSG_PING` (`0x01`) | STM32 nháy LED $\rightarrow$ `[RESP] PONG (STM32 Alive)\r\n` |
| `STOP` hoặc `EMG` | Không | `AGV_MSG_EMERGENCY_STOP` (`0x02`) | Phanh cứng motor $\rightarrow$ `[RESP] EMERGENCY STOP SENT\r\n` |
| `STEP_TEST <motor> <pwm> <ms>` | `motor`: `L`/`R`<br>`pwm`: `-1000..1000`<br>`ms`: thời gian | `AGV_MSG_DIAG_STEP_TEST` (`0x20`) | Kích xung bước nhảy. STM32 stream liên tục `DATA:<ms>,<s_int.s_frac>,<pwm>,<pct>,<ticks>,<rpm>,<rads>` qua BLE về PC |
| `MONITOR_IR [ms]` | `ms`: thời gian (mặc định 15000) | `AGV_MSG_DIAG_LINE_TEST` (`0x26`, `mode=0`, `rpm=0`) | Giám sát 5 mắt IR. STM32 stream `DATA:LINE,<ms>,<raw0..4>,<norm0..4>,<err>,<seen>` qua BLE về PC |
| `LINE_CALIB [ms]` | `ms`: thời gian (mặc định 10000) | `AGV_MSG_DIAG_LINE_TEST` (`0x26`, `mode=1`, `rpm=0`) | Quét min/max IR bàn test. STM32 stream dữ liệu học min/max qua BLE về PC |
| `LINE_TEST <mode> <ms> <rpm>` | `mode`: 0:SENSOR, 1:CALIB, 2:TRACK, 3:STOP | `AGV_MSG_DIAG_LINE_TEST` (`0x26`) | Kích hoạt bài test IR tổng quát kèm vận tốc cơ sở khi bám line test |
| `GET_ENC` | Không | `AGV_MSG_DIAG_GET_ENC` (`0x23`) | Đọc số xung encoder $\rightarrow$ `[ENCODER] L: %ld \| R: %ld\r\n` |
| `RESET_ENC` | Không | `AGV_MSG_DIAG_RESET_ENC` (`0x24`) | Đặt lại xung encoder về 0 $\rightarrow$ `[RESP] ENCODER RESET OK\r\n` |
| `SET_SPEED <left> <right>` | `left`, `right`: `-1000..1000` | `AGV_MSG_DIAG_SET_SPEED` (`0x21`) | Đặt mức PWM trực tiếp 2 bánh |
| `SET_RPM <left> <right>` | `left`, `right`: RPM float | `AGV_MSG_DIAG_SET_RPM` (`0x22`) | Đặt vận tốc vòng kín RPM |
| `PI_TUNE <kp> <ki> <kff>` | Hệ số float | `AGV_MSG_DIAG_PI_TUNE` (`0x25`) | Cập nhật bộ hệ số PI động cơ |
| `ROUTE_LOAD <rpm> <actions>` | `rpm`: float<br>`actions`: `F,R,L,S` | `AGV_MSG_ROUTE_LOAD` (`0x10`) | Nạp danh sách ngã rẽ |
| `ROUTE_START` | Không | `AGV_MSG_ROUTE_START` (`0x11`) | Bắt đầu chạy lộ trình $\rightarrow$ stream `[MISSION] ...` |
| `ROUTE_CANCEL` | Không | `AGV_MSG_ROUTE_CANCEL` (`0x12`) | Hủy lộ trình đang chạy |
| `ROUTE_TEST` | Không | Gửi liên tiếp `ROUTE_LOAD` mẫu và `ROUTE_START` | Chạy nhanh bài test lộ trình 4 bước |
| `HELP` | Không | Không | In bảng trợ giúp cú pháp đầy đủ |

### 5.3 Định Dạng Dữ Liệu Phản Hồi Từ STM32 -> ESP32 -> BLE Client
- **Telemetry Dò Line**:
  `DATA:LINE,<ms>,<raw0>,<raw1>,<raw2>,<raw3>,<raw4>,<norm0>,<norm1>,<norm2>,<norm3>,<norm4>,<err>,<seen>\r\n`
- **Telemetry Động Cơ**:
  `DATA:<ms>,<time_s>,<pwm>,<pwm_pct>,<delta_ticks>,<speed_rpm>,<speed_rads>\r\n`
- **Thông Báo Trạng Thái / Log**:
  `[STM32 LOG] ...\r\n`
  `[ACK] MsgID: 0x%02X, Status: %s (Code: %d)\r\n`
  `[MISSION] State: %s, Step: %d\r\n`
  `[ENCODER] Left: %ld | Right: %ld\r\n`

---

## 6. Thiết Kế Bộ Công Cụ Python PC (`agv_esp32/tools/agv_ble_controller.py`)

Công cụ Python được thiết kế đa năng, tích hợp 4 tính năng chính:

```
┌────────────────────────────────────────────────────────────────────────┐
│ AGV BLE WIRELESS WORKBENCH (Python Bleak Async Engine)                 │
├──────────────────┬──────────────────┬──────────────────┬───────────────┤
│ [1. REPL Prompt] │ [2. IR Monitor]  │ [3. Calibrate]   │ [4. Step Plot]│
│ - Gõ lệnh text   │ - Parse DATA:LINE│ - Min/Max scan   │ - Thu thập RPM│
│ - Xem log màu    │ - Vẽ bar Unicode │ - Kiểm tra D-Rang│ - Vẽ matplotlib│
│ - Dừng khẩn cấp  │ - e_line sai số  │ - Xuất header .h │ - Lưu file CSV│
└──────────────────┴──────────────────┴──────────────────┴───────────────┘
```

1. **Chế độ `monitor_ir`**:
   - Gửi lệnh `MONITOR_IR <duration>` qua BLE.
   - Bắt các dòng `DATA:LINE,...`, xóa màn hình Terminal và in giao diện trực quan:
     ```text
     ==================================================
       AGV STM32 IR ARRAY MONITOR (BLE WIRELESS)
       Elapsed: 1250 ms | Line Detected: YES [ON TRACK]
       Centroid Error (e_line): -150 (Lệch phải nhẹ)
     ==================================================
       IR_R2 (Ch0) |████████████░░░░░░░░░| Norm: 600 | Raw ADC: 2450
       IR_R1 (Ch1) |█████████████████████| Norm: 950 | Raw ADC: 3820
       IR_MID(Ch2) |█████████████████████| Norm: 980 | Raw ADC: 3910
       IR_L1 (Ch3) |████████░░░░░░░░░░░░░| Norm: 400 | Raw ADC: 1650
       IR_L2 (Ch4) |░░░░░░░░░░░░░░░░░░░░░| Norm:  50 | Raw ADC:  210
     ==================================================
     ```
2. **Chế độ `calibrate_line`**:
   - Gửi lệnh `LINE_CALIB <duration>` qua BLE.
   - Hướng dẫn người dùng lia giấy trắng và vạch đen.
   - Tính toán `min_raw[5]` và `max_raw[5]`, đối chiếu $\Delta_i \ge 100$.
   - Khi hết thời gian, in bảng kết quả và hỏi người dùng có ghi đè trực tiếp vào `agv_stm32/Application/Inc/app_line_calib_defaults.h` hay không.
3. **Chế độ `step_test` & Vẽ đồ thị**:
   - Gửi lệnh `STEP_TEST <motor> <pwm> <ms>` qua BLE.
   - Bắt các dòng `DATA:<ms>,...`, trích xuất `time_ms` và `speed_rpm`.
   - Lưu vào file CSV `step_response_ble.csv`.
   - Nếu có cờ `--plot`, tự động mở cửa sổ đồ thị Matplotlib hiển thị đặc tuyến tăng tốc, vận tốc đỉnh và vận tốc xác lập.

---

## 7. Code Style & Quy Chuẩn Lập Trình (ESP-IDF v6.x)

1. **Tương thích ESP-IDF v6.x**:
   - Khởi tạo UART qua `uart_driver_install()` và cấu hình chân qua `uart_set_pin()`.
   - Cấu hình NimBLE NUS trong `sdkconfig` đảm bảo MTU $\ge 256$ byte (`CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256`).
2. **Zero-malloc trong RX Loop**:
   - Không gọi cấp phát động trong vòng lặp đọc byte UART hoặc callback BLE.
   - Dùng buffer tĩnh và ringbuffer có sẵn của ESP-IDF UART driver.
3. **Không chặn (Non-blocking)**:
   - Các luồng nhận UART và BLE chạy bất đồng bộ trong các FreeRTOS Task riêng biệt.

---

## 8. Boundaries (Ranh Giới Phát Triển)

- **LUÔN LUÔN (Always):**
  - Luôn kiểm tra tính toàn vẹn CRC16-CCITT trên tất cả các khung truyền nhận từ UART2 trước khi phân tích nội dung payload.
  - Phải nối chung chân Mass (GND) giữa ESP32 và STM32 khi kết nối cáp tín hiệu vật lý.
  - Giữ nguyên vẹn mã nguồn `comms_protocol.c` và `comms_protocol.h` đồng bộ 100% giữa STM32 và ESP32.
  - Xử lý bất đồng bộ (non-blocking) trong các callback ngắt hoặc luồng BLE, không gọi hàm trễ `vTaskDelay` làm nghẽn stack NimBLE.
- **HỎI TRƯỚC (Ask first):**
  - Thay đổi cấu hình chân phần cứng UART (GPIO16/17) hoặc tốc độ baudrate 115200.
  - Thay đổi UUID chuẩn của Nordic UART Service (NUS).
  - Bổ sung thêm mã lệnh hoặc thay đổi cấu trúc struct trong `comms_protocol.h`.
- **TUYỆT ĐỐI KHÔNG (Never):**
  - Không gửi chuỗi văn bản thuần (raw ASCII) trực tiếp xuống UART STM32.
  - Không gọi cấp phát động (`malloc`/`calloc`) bên trong vòng lặp nhận byte UART chu kỳ cao.
  - Không nuốt lỗi CRC hoặc bỏ qua frame hỏng một cách âm thầm mà không log cảnh báo.

---

## 9. Success Criteria (Tiêu Chí Thành Công & Nghiệm Thu)

- [ ] **SC-1 (Biên dịch)**: Dự án `agv_esp32` biên dịch thành công 100% không cảnh báo lỗi trên **ESP-IDF v6.x (v6.0.2)** (`idf.py build`).
- [ ] **SC-2 (Component Structure)**: Component `comms_protocol` được liên kết chuẩn xác qua CMake, không phụ thuộc ngoại vi HAL.
- [ ] **SC-3 (Binary Frame Parsing)**: `uart_service` nhận stream byte từ UART2, loại bỏ byte rác ngoài luồng, và parse thành công các gói tin `AGV_MSG_ACK`, `AGV_MSG_PONG`, `AGV_MSG_LOG`, `AGV_MSG_MISSION_STATUS`, `AGV_MSG_DIAG_ENC_DATA`.
- [ ] **SC-4 (Physical Ping Test)**: Bấm nút BOOT (GPIO0) trên ESP32 $\rightarrow$ gửi `AGV_MSG_PING` xuống STM32 $\rightarrow$ LED PC13 trên STM32 nháy và trả lời `PONG`.
- [ ] **SC-5 (BLE Command Dispatching)**: Client gửi lệnh qua BLE $\rightarrow$ ESP32 đóng gói đúng các lệnh binary tương ứng xuống STM32.
- [ ] **SC-6 (Emergency Stop Safety)**: Client gửi chuỗi `STOP` qua BLE $\rightarrow$ ESP32 phát ngay lập tức `AGV_MSG_EMERGENCY_STOP` $\rightarrow$ STM32 phanh cứng 2 bánh.
- [ ] **SC-7 (Log Stream Relay)**: Toàn bộ dòng log phát ra từ STM32 (`AGV_MSG_LOG`) được ESP32 chuyển tiếp trọn vẹn lên console Python Client theo thời gian thực.
- [ ] **SC-8 (Step Response Chẩn Đoán qua BLE)**: Chạy `step` qua BLE $\rightarrow$ nhận được dữ liệu `DATA:...`, hiển thị vận tốc đỉnh, vận tốc xác lập và lưu file CSV / vẽ đồ thị đặc tuyến.
- [ ] **SC-9 (Monitor IR Chẩn Đoán qua BLE)**: Chạy `monitor_ir` qua BLE $\rightarrow$ hiển thị trực quan thanh bar 5 mắt cảm biến và $e_{line}$ liên tục ở 20Hz.
- [ ] **SC-10 (Calibrate Line Chẩn Đoán qua BLE)**: Chạy `calibrate` qua BLE $\rightarrow$ quét min/max quang học, kiểm tra $\Delta \ge 100$, và tạo file `app_line_calib_defaults.h`.

---

## 10. Open Questions (Câu Hỏi Mở)

Hiện tại toàn bộ yêu cầu về chẩn đoán không dây (Step Response, IR Monitor, Calibrate Line) và điều khiển lộ trình đã được đặc tả chi tiết và khép kín. Không còn câu hỏi chặn kỹ thuật (blocker).
