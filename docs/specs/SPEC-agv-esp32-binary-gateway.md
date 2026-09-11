# Spec: AGV ESP32 Binary UART Gateway & Wireless BLE Bench-Test Client

## Phase 0: Capability Map

| Module ID | Responsibility | Depends On |
|---|---|---|
| `comms-protocol` | Mã nguồn C thuần giao thức nhị phân (`comms_protocol.c/.h`): CRC16-CCITT, byte-stream parser FSM, pack/unpack frame. | — |
| `uart-service` | Driver UART2 (GPIO16/17, 115200 8N1), FreeRTOS RX Task phân tích khung truyền nhị phân, bộ đệm gửi TX, callback dispatch frame cho ứng dụng. | `comms-protocol` |
| `ble-command-bridge` | Module `ble_cmd_dispatcher`: Biên dịch chuỗi text từ BLE NUS sang binary frame cho STM32; format phản hồi từ STM32 (ACK, PONG, LOG, STATUS, ENC) thành text gửi lại BLE NUS. | `uart-service`, `ble_nus` |
| `tools-ble-client` | Script Python tương tác trên PC (`agv_esp32/tools/agv_ble_controller.py`) dùng thư viện `bleak` để quét BLE, kết nối tới `AGV_ESP32`, gửi lệnh text và hiển thị telemetry/log thời gian thực. | `ble-command-bridge` |

**Build order:** `comms-protocol` → `uart-service` → `ble-command-bridge` → `tools-ble-client`

---

## 1. Objective (Mục Tiêu)

Xây dựng tầng firmware truyền thông nhị phân trên **ESP32 (ESP-IDF v6.x / v6.0.2)** để kết nối đồng bộ với vi điều khiển chấp hành **STM32F103C8T6** theo chuẩn giao thức nhị phân `comms_protocol` (ADR-003), đồng thời cung cấp giao diện điều khiển cấp cao qua **Bluetooth Low Energy (NimBLE Nordic UART Service - NUS)**.

Song song đó, cung cấp một script Python trên máy tính (`agv_esp32/tools/agv_ble_controller.py`) cho phép kỹ sư vận hành kết nối BLE không dây, gửi các lệnh text cấp cao để kiểm tra phần cứng (Ping, Step Test động cơ, đọc Encoder, nạp & chạy lộ trình mẫu, Dừng khẩn cấp) và nhận log/trạng thái từ STM32 hiển thị trực quan theo thời gian thực trên Terminal.

### User Stories & Acceptance Criteria

1. **US-1: Kiểm tra kết nối 2 chiều (Ping/Pong)**
   - *Khi* người dùng gửi lệnh `PING` qua BLE (hoặc ấn nút cứng BOOT trên ESP32).
   - *Thì* ESP32 đóng gói `AGV_MSG_PING` (ID `0x01`) gửi qua UART2 tới STM32; STM32 nháy LED PC13 và trả lời `AGV_MSG_PONG` (ID `0x81`); ESP32 gửi thông báo `[RESP] PONG (STM32 Alive)\r\n` về client BLE.
2. **US-2: Chạy thử đáp ứng bước động cơ (Step Test)**
   - *Khi* người dùng nhập `STEP_TEST <motor> <pwm> <ms>` (ví dụ: `STEP_TEST L 500 1000` hoặc `STEP_TEST R -400 1500`).
   - *Thì* ESP32 parse tham số, kiểm tra phạm vi (`pwm` $\in [-1000, 1000]$, `duration_ms` $> 0$), đóng gói `AGV_MSG_DIAG_STEP_TEST` (ID `0x20`) gửi sang STM32, và chuyển tiếp các frame `AGV_MSG_ACK` cùng toàn bộ stream log `AGV_MSG_LOG` về BLE.
3. **US-3: Đọc và Reset xung Encoder**
   - *Khi* người dùng gửi `GET_ENC`, ESP32 gửi `AGV_MSG_DIAG_GET_ENC` (ID `0x23`), đón nhận frame `AGV_MSG_DIAG_ENC_DATA` (ID `0xA3`) và hiển thị: `[ENCODER] Left: <ticks> | Right: <ticks>\r\n`.
   - *Khi* người dùng gửi `RESET_ENC`, ESP32 gửi `AGV_MSG_DIAG_RESET_ENC` (ID `0x24`) và trả về `[RESP] ENCODER RESET OK\r\n`.
4. **US-4: Nạp và kích hoạt lộ trình tự hành (Route Mission)**
   - *Khi* người dùng gửi `ROUTE_LOAD <rpm> <actions...>` (ví dụ: `ROUTE_LOAD 80.0 F,R,F,S` với F=Forward, L=Left, R=Right, S=Stop), ESP32 đóng gói `AGV_MSG_ROUTE_LOAD` (ID `0x10`).
   - *Khi* người dùng gửi `ROUTE_START`, ESP32 phát `AGV_MSG_ROUTE_START` (ID `0x11`).
   - *Khi* xe chạy bám line, các gói `AGV_MSG_MISSION_STATUS` (ID `0x90`) được ESP32 format và bắn liên tục về BLE: `[MISSION] State: RUNNING, Step: 1\r\n` ... `[MISSION] State: COMPLETED, Step: 4\r\n`.
5. **US-5: Dừng phanh khẩn cấp (Emergency Stop)**
   - *Khi* người dùng gửi `STOP` hoặc `EMG`, ESP32 phát ngay lập tức `AGV_MSG_EMERGENCY_STOP` (ID `0x02`), STM32 phanh cứng động cơ và trả về ACK.
6. **US-6: Kênh log đo xa không dây (Wireless Log Relay)**
   - *Khi* STM32 phát sinh bất kỳ log debug nào qua `AGV_LOG(...)`, frame `AGV_MSG_LOG` (ID `0x99`) được gửi sang ESP32 qua UART2; ESP32 bọc vào tiền tố `[STM32 LOG]` và notify ngay về BLE NUS.

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
- **PC Test Client**:
  - Python 3.10+ (chạy trên Windows/Linux/macOS).
  - Thư viện: `bleak >= 0.21.0`, `asyncio`.

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
# Cài đặt thư viện bleak
pip install bleak

# Chạy script tương tác dòng lệnh BLE
python agv_esp32/tools/agv_ble_controller.py

# Hoặc chỉ định trực tiếp địa chỉ MAC nếu có nhiều thiết bị BLE xung quanh
python agv_esp32/tools/agv_ble_controller.py --address "XX:XX:XX:XX:XX:XX"
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
    │   │       └── uart_service.h    # API gửi frame nhị phân & đăng ký callback nhận frame
    │   ├── ble_nus/                  # [Component 3] NimBLE Nordic UART Service
    │   │   ├── CMakeLists.txt
    │   │   ├── ble_nus.c             # GAP advertising, GATT table, notify callback
    │   │   └── include/
    │   │       └── ble_nus.h         # ble_nus_send_str(), rx_callback
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
    └── tools/                        # Công cụ tương tác BLE trên máy tính
        └── agv_ble_controller.py     # Script CLI tương tác qua Bleak
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
- **SOF**: `0xAA 0x55`
- **CRC16**: CRC16-CCITT (Poly `0x1021`, Init `0xFFFF`) tính trên `[MSG_ID] + [LENGTH] + [PAYLOAD]`.

### 5.2 Interface Component `uart_service` (Tầng UART Nhị Phân)

```c
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "comms_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*agv_frame_rx_cb_t)(const agv_frame_t *frame, void *user_data);

typedef struct {
    uart_port_t uart_port;            /**< Mặc định: UART_NUM_2 */
    gpio_num_t tx_pin;                /**< Mặc định: GPIO_NUM_17 */
    gpio_num_t rx_pin;                /**< Mặc định: GPIO_NUM_16 */
    int baud_rate;                    /**< Mặc định: 115200 */
    size_t rx_buffer_size;            /**< Driver ringbuffer size (>= 1024) */
    size_t tx_buffer_size;            /**< Driver TX buffer size (>= 1024) */
    agv_frame_rx_cb_t frame_callback; /**< Callback nhận frame nhị phân hoàn chỉnh */
    void *user_data;                  /**< Context cho callback */
} uart_service_config_t;

/**
 * @brief Khởi tạo UART driver và khởi chạy FreeRTOS RX Task giải mã frame nhị phân
 */
esp_err_t uart_service_init(const uart_service_config_t *config);

/**
 * @brief Gửi một gói tin nhị phân chuẩn agv_frame_t sang STM32
 */
int uart_service_send_frame(const agv_frame_t *frame);

/**
 * @brief Đóng gói và gửi payload cụ thể sang STM32
 */
int uart_service_send_msg(uint8_t msg_id, const void *payload, uint8_t payload_len);

/**
 * @brief Giải phóng tài nguyên UART service
 */
esp_err_t uart_service_deinit(void);

#ifdef __cplusplus
}
#endif
```

### 5.3 Bảng Ánh Xạ Lệnh BLE Cấp Cao (`ble_cmd_dispatcher`)

Người dùng gõ lệnh văn bản (ASCII text) trên terminal BLE client; module `ble_cmd_dispatcher` phân tích và chuyển đổi:

| Lệnh Text BLE (từ Client) | Tham Số | Gói Nhị Phân Sinh Ra | Mô Tả Hành Vi | Định Dạng Phản Hồi Trả Về BLE |
|---|---|---|---|---|
| `PING` | Không | `AGV_MSG_PING` (`0x01`) | Kiểm tra sống/chết | Chờ `PONG` $\rightarrow$ `[RESP] PONG (STM32 Alive)\r\n` |
| `STOP` hoặc `EMG` | Không | `AGV_MSG_EMERGENCY_STOP` (`0x02`) | Phanh khẩn cấp lập tức | `[RESP] EMERGENCY STOP SENT\r\n` |
| `STEP_TEST <motor> <pwm> <ms>` | `motor`: `L`/`R` hoặc `0`/`1`<br>`pwm`: `-1000..1000`<br>`ms`: thời gian | `AGV_MSG_DIAG_STEP_TEST` (`0x20`) | Chạy thử 1 động cơ với PWM | `[RESP] STEP_TEST sent (Motor: %s, PWM: %d, Time: %lums)\r\n` |
| `SET_SPEED <left> <right>` | `left`: `-1000..1000`<br>`right`: `-1000..1000` | `AGV_MSG_DIAG_SET_SPEED` (`0x21`) | Đặt mức PWM trực tiếp 2 bánh | `[RESP] SPEED set (%d, %d)\r\n` |
| `SET_RPM <left> <right>` | `left`: RPM float<br>`right`: RPM float | `AGV_MSG_DIAG_SET_RPM` (`0x22`) | Đặt vận tốc vòng kín PI | `[RESP] RPM set (%.1f, %.1f)\r\n` |
| `GET_ENC` | Không | `AGV_MSG_DIAG_GET_ENC` (`0x23`) | Đọc số xung encoder | Chờ frame `0xA3` $\rightarrow$ `[ENCODER] L: %ld \| R: %ld\r\n` |
| `RESET_ENC` | Không | `AGV_MSG_DIAG_RESET_ENC` (`0x24`) | Reset bộ đếm xung | `[RESP] RESET_ENC sent\r\n` |
| `PI_TUNE <kp> <ki> <kff>` | Hệ số float | `AGV_MSG_DIAG_PI_TUNE` (`0x25`) | Cập nhật hệ số PI vòng kín | `[RESP] PI_TUNE sent (Kp: %.2f, Ki: %.2f, Kff: %.2f)\r\n` |
| `LINE_CALIB [mode] [time]` | `mode`: 0..3 (mặc định 1: CALIB)<br>`time`: ms (mặc định 3000) | `AGV_MSG_DIAG_LINE_TEST` (`0x26`) | Hiệu chuẩn quang học IR | `[RESP] LINE_CALIB sent (Mode: %d, Time: %lums)\r\n` |
| `ROUTE_LOAD <rpm> <actions>` | `rpm`: ví dụ `80.0`<br>`actions`: `F,R,L,S` hoặc `1,3,2,0` | `AGV_MSG_ROUTE_LOAD` (`0x10`) | Nạp danh sách ngã rẽ | `[RESP] ROUTE LOADED (%d steps, Target: %.1f RPM)\r\n` |
| `ROUTE_START` | Không | `AGV_MSG_ROUTE_START` (`0x11`) | Bắt đầu chạy lộ trình | `[RESP] ROUTE_START sent\r\n` |
| `ROUTE_CANCEL` | Không | `AGV_MSG_ROUTE_CANCEL` (`0x12`) | Hủy lộ trình xe đang chạy | `[RESP] ROUTE_CANCEL sent\r\n` |
| `ROUTE_TEST` | Không | Gửi liên tiếp `ROUTE_LOAD` mẫu và `ROUTE_START` | Chạy nhanh lộ trình 4 bước | `[RESP] ROUTE_TEST triggered (4 steps)\r\n` |
| `HELP` | Không | Không | Trợ giúp danh sách lệnh | In bảng danh sách lệnh được hỗ trợ |

### 5.4 Định Dạng Sự Kiện & Log Tự Động Từ STM32 Lên BLE
Mỗi khi nhận được frame từ STM32 qua UART2, `ble_cmd_dispatcher` định dạng thành chuỗi ASCII và gọi `ble_nus_send_str(...)`:
- **`AGV_MSG_ACK` (`0x80`)**:
  ```text
  [ACK] MsgID: 0x%02X, Status: %s (Code: %d)\r\n
  ```
  *(Status: 0: OK, 1: ERR_CRC, 2: ERR_UNKNOWN_CMD, 3: ERR_INVALID_ARGS, 4: ERR_BUSY)*
- **`AGV_MSG_PONG` (`0x81`)**:
  ```text
  [RESP] PONG (STM32 Alive)\r\n
  ```
- **`AGV_MSG_LOG` (`0x99`)**:
  ```text
  [STM32 LOG] <nội dung log nguyên bản>\r\n
  ```
- **`AGV_MSG_MISSION_STATUS` (`0x90`)**:
  ```text
  [MISSION] State: %s, Step: %d\r\n
  ```
  *(State: IDLE, RUNNING, COMPLETED, ERROR_LOST, ERROR_TIMEOUT)*
- **`AGV_MSG_DIAG_ENC_DATA` (`0xA3`)**:
  ```text
  [ENCODER] Left: %ld | Right: %ld\r\n
  ```

---

## 6. Code Style & Quy Chuẩn Lập Trình (ESP-IDF v6.x)

### 6.1 Nguyên Tắc Thiết Kế Firmware
1. **Tương thích ESP-IDF v6.x**: Sử dụng các header driver chuẩn mới (`driver/uart.h`, `driver/gpio.h`, `nimble/nimble_port.h`), kiểm tra mã trả về qua macro `ESP_ERROR_CHECK()` hoặc xử lý `esp_err_t`.
2. **100% Zero-malloc trong Task UART**: Tuyệt đối không gọi `malloc()`, `calloc()`, hay `free()` trong vòng lặp đọc byte từ UART; mọi frame và buffer giải mã đều sử dụng bộ đệm tĩnh (`static agv_parser_t`, `static agv_frame_t`).
3. **Phân Tách Luồng & Hàng Đợi (FreeRTOS Multi-Tasking)**:
   - Task `uart_rx_task` (Priority 10, Stack 4KB): Chỉ đọc luồng byte từ UART driver ringbuffer, chạy máy trạng thái FSM `agv_parser_feed_byte`, khi trọn vẹn frame thì gọi callback chuyển giao.
   - Callback của NimBLE BLE RX (`on_ble_data_received`): Copy chuỗi vào buffer tĩnh và chuyển giao cho `ble_cmd_dispatcher` xử lý; không block luồng BLE quá 5ms.

### 6.2 Ví Dụ Code Chuẩn của Task UART RX Giải Mã Nhị Phân
```c
static void uart_rx_task(void *arg) {
    uart_service_t *svc = (uart_service_t *)arg;
    uint8_t rx_chunk[64];
    agv_parser_t parser;
    agv_frame_t rx_frame;

    agv_parser_init(&parser);
    ESP_LOGI(TAG, "UART binary RX task started on core %d", xPortGetCoreID());

    while (svc->is_running) {
        // Đọc không blocking với timeout ngắn 20ms
        int len = uart_read_bytes(svc->cfg.uart_port, rx_chunk, sizeof(rx_chunk), pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                agv_parse_status_t status = agv_parser_feed_byte(&parser, rx_chunk[i], &rx_frame);
                if (status == AGV_PARSE_OK) {
                    if (svc->cfg.frame_callback) {
                        svc->cfg.frame_callback(&rx_frame, svc->cfg.user_data);
                    }
                } else if (status == AGV_PARSE_ERR_CRC) {
                    ESP_LOGW(TAG, "CRC16 mismatch on incoming STM32 frame (msg_id=0x%02X)", parser.msg_id);
                }
            }
        }
    }
    vTaskDelete(NULL);
}
```

---

## 7. Testing Strategy (Chiến Lược Kiểm Thử & Nghiệm Thu)

Hệ thống được kiểm thử qua 4 cấp độ:

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Unit Tests Protocol (PC Python / C Runner)               │
│    - Kiểm thử thuật toán CRC16-CCITT chuẩn                 │
│    - Kiểm thử Parser FSM: byte rác, cắt vụn stream byte     │
├─────────────────────────────────────────────────────────────┤
│ 2. Loopback & Component Build Verification (ESP32)          │
│    - Biên dịch sạch sẽ 100% không warning trên ESP-IDF v6.x  │
│    - Kiểm tra đăng ký component comms_protocol & uart_service│
├─────────────────────────────────────────────────────────────┤
│ 3. Hardware Integration Verification (ESP32 <-> STM32)      │
│    - Đấu nối vật lý GPIO16/17 <-> PA9/PA10, chung GND       │
│    - Bấm nút BOOT (GPIO0) -> LED PC13 trên STM32 nháy       │
├─────────────────────────────────────────────────────────────┤
│ 4. Wireless End-to-End Test via Python Bleak Client (PC)    │
│    - Quét tìm 'AGV_ESP32', kết nối và đăng ký Notifications │
│    - Chạy chuỗi lệnh PING, STEP_TEST, GET_ENC, STOP         │
│    - Kiểm tra tốc độ stream log thời gian thực qua BLE      │
└─────────────────────────────────────────────────────────────┘
```

### Chi Tiết Công Cụ Client PC: `agv_esp32/tools/agv_ble_controller.py`
- Tự động scan tìm thiết bị có tên quảng bá `AGV_ESP32` hoặc Service UUID `6E400001-...`.
- Đăng ký nhận thông báo (Notification) trên characteristic `6E400003-...` và in ra màn hình console với mã màu phân biệt:
  - `[STM32 LOG]` màu vàng/cyan.
  - `[ACK]` màu xanh lá.
  - `[ENCODER]` màu xanh dương.
  - `[MISSION]` màu tím nổi bật.
- Cung cấp vòng lặp nhập lệnh không đồng bộ (Asynchronous Prompt Loop), hỗ trợ lịch sử lệnh và tự động ngắt khi gõ `quit` hoặc `exit`.

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
  - Không gửi chuỗi văn bản thuần (raw ASCII) trực tiếp xuống UART STM32 (STM32 chỉ chấp nhận binary frame có SOF `0xAA 0x55` và CRC16).
  - Không gọi cấp phát động (`malloc`/`calloc`) bên trong vòng lặp nhận byte UART chu kỳ cao.
  - Không nuốt lỗi CRC hoặc bỏ qua frame hỏng một cách âm thầm mà không log cảnh báo.

---

## 9. Success Criteria (Tiêu Chí Thành Công & Nghiệm Thu)

- [ ] **SC-1 (Biên dịch)**: Dự án `agv_esp32` biên dịch thành công 100% không cảnh báo lỗi trên **ESP-IDF v6.x (v6.0.2)** (`idf.py build`).
- [ ] **SC-2 (Component Structure)**: Component `comms_protocol` được liên kết chuẩn xác qua CMake, không phụ thuộc ngoại vi HAL.
- [ ] **SC-3 (Binary Frame Parsing)**: `uart_service` nhận stream byte từ UART2, loại bỏ byte rác ngoài luồng, và parse thành công các gói tin `AGV_MSG_ACK`, `AGV_MSG_PONG`, `AGV_MSG_LOG`, `AGV_MSG_MISSION_STATUS`, `AGV_MSG_DIAG_ENC_DATA`.
- [ ] **SC-4 (Physical Ping Test)**: Bấm nút BOOT (GPIO0) trên ESP32 $\rightarrow$ gửi `AGV_MSG_PING` xuống STM32 $\rightarrow$ LED PC13 trên STM32 nháy và trả lời `PONG`.
- [ ] **SC-5 (BLE Command Dispatching)**: Client gửi chuỗi `STEP_TEST L 500 1000` qua BLE $\rightarrow$ ESP32 đóng gói đúng `AGV_MSG_DIAG_STEP_TEST` gửi sang STM32.
- [ ] **SC-6 (Emergency Stop Safety)**: Client gửi chuỗi `STOP` qua BLE $\rightarrow$ ESP32 phát ngay lập tức `AGV_MSG_EMERGENCY_STOP` $\rightarrow$ STM32 phanh cứng 2 bánh.
- [ ] **SC-7 (Log Stream Relay)**: Toàn bộ dòng log phát ra từ STM32 (`AGV_MSG_LOG`) được ESP32 chuyển tiếp trọn vẹn lên console Python Client theo thời gian thực.
- [ ] **SC-8 (Python BLE Tool)**: Script `agv_ble_controller.py` hoạt động ổn định trên PC, tự động kết nối BLE, cung cấp giao diện nhập lệnh tiện lợi và phân biệt màu sắc log.

---

## 10. Open Questions (Câu Hỏi Mở)

Hiện tại toàn bộ thông số kỹ thuật, baudrate, chân cắm GPIO, từ điển mã lệnh và cấu trúc frame đã được thống nhất hoàn toàn. Không còn câu hỏi chặn kỹ thuật (blocker).
