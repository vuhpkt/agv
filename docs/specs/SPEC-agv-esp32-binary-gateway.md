# Spec: AGV ESP32 Binary UART Gateway & Wireless BLE Bench-Test Client

## Phase 0: Capability Map

| Module ID | Responsibility | Depends On |
|---|---|---|
| `comms-protocol` | Mã nguồn C thuần giao thức nhị phân (`comms_protocol.c/.h`): CRC16-CCITT, byte-stream parser FSM, pack/unpack frame. | — |
| `uart-service` | Driver UART2 (GPIO16/17, 115200 8N1), FreeRTOS RX Task phân tích khung truyền nhị phân, hàng đợi TX, callback dispatch frame. | `comms-protocol` |
| `ble-command-bridge` | Bộ biên dịch lệnh text cấp cao từ BLE NUS sang binary frame cho STM32, định dạng phản hồi nhị phân (ACK/LOG/STATUS/ENC) thành chuỗi text trả về BLE NUS. | `uart-service`, `ble_nus` |
| `tools-ble-client` | Script Python tương tác trên máy tính (Windows) dùng thư viện `bleak` để quét BLE, kết nối tới `AGV_ESP32`, gửi lệnh text và hiển thị telemetry/log thời gian thực. | `ble-command-bridge` |

**Build order:** `comms-protocol` → `uart-service` → `ble-command-bridge` → `tools-ble-client`

---

## 1. Objective (Mục Tiêu)

Xây dựng tầng firmware truyền thông nhị phân trên ESP32 (ESP-IDF) để kết nối đồng bộ với STM32F103 (Motion Controller) theo chuẩn giao thức `comms_protocol` (ADR-003), đồng thời cung cấp giao diện điều khiển cấp cao qua Bluetooth Low Energy (BLE Nordic UART Service). 

Song song đó, cung cấp một script Python trên máy tính (`tools/agv_ble_controller.py`) cho phép kỹ sư vận hành kết nối BLE không dây, gửi các lệnh text cấp cao để kiểm tra phần cứng (Ping, Step Test động cơ, đọc Encoder, nạp & chạy lộ trình mẫu, Dừng khẩn cấp) và nhận log/trạng thái từ STM32 hiển thị trực quan trên Terminal.

### User Stories & Use Cases
1. **Kiểm tra kết nối (Ping/Pong)**: Người dùng nhập `PING` trên Terminal Python -> ESP32 gửi frame `AGV_MSG_PING` nhị phân -> STM32 nháy LED PC13 và trả lời `AGV_MSG_PONG` -> ESP32 bắn `OK: PONG` về Python.
2. **Kiểm tra động cơ (Step Test)**: Người dùng nhập `STEP_TEST <motor> <pwm> <ms>` (ví dụ `STEP_TEST L 500 1000`) -> ESP32 đóng gói `AGV_MSG_DIAG_STEP_TEST` -> STM32 chạy bánh trái PWM 500 trong 1s, trả về ACK và log -> Python nhận kết quả `[ACK] OK`, `[STM32 LOG] ...`.
3. **Đọc vị trí Encoder**: Người dùng nhập `GET_ENC` -> ESP32 gửi `AGV_MSG_DIAG_GET_ENC` -> STM32 trả về `AGV_MSG_DIAG_ENC_DATA` -> Python hiển thị `[ENCODER] Left: 1320 | Right: 1315`.
4. **Nạp & chạy lộ trình mẫu (Route Test)**: Người dùng nhập `ROUTE_TEST` -> ESP32 đóng gói `AGV_MSG_ROUTE_LOAD` (gồm danh sách bước FORWARD, RIGHT, FORWARD, STOP) và `AGV_MSG_ROUTE_START` -> STM32 xác nhận ACK và chuyển trạng thái `MISSION_STATUS: RUNNING` -> STM32 bám line, vượt ngã rẽ và khi tới đích trả về `MISSION_STATUS: COMPLETED` -> Python hiển thị trạng thái hoàn thành.
5. **Dừng khẩn cấp (Emergency Stop)**: Người dùng nhập `STOP` hoặc `EMG` -> ESP32 lập tức phát `AGV_MSG_EMERGENCY_STOP` -> STM32 phanh cứng motor và trả về ACK.

---

## 2. Tech Stack (Công Nghệ & Môi Trường)

- **Vi điều khiển Gateway**: ESP32 (Dual-Core Xtensa @ 240MHz, 4MB Flash, 520KB SRAM).
- **Framework Firmware**: ESP-IDF v5.x (C99/C11, FreeRTOS, NimBLE).
- **Giao tiếp ngoại vi**:
  - UART2: TX = GPIO17, RX = GPIO16, 115200 bps, 8 data bits, no parity, 1 stop bit (8N1).
  - BLE: NimBLE Nordic UART Service (NUS), MTU mặc định 23-256 byte, Service UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`.
  - BOOT Button: GPIO0 (Active-LOW, kích hoạt gửi PING test nhanh).
- **Client kiểm thử trên PC**:
  - Python 3.10+ (chạy trên Windows 10/11).
  - Thư viện: `bleak >= 0.21.0`, `asyncio`.

---

## 3. Commands (Lệnh Thực Thi)

### Firmware (ESP-IDF)
```bash
# Cấu hình dự án (nếu cần)
idf.py set-target esp32

# Biên dịch mã nguồn
idf.py build

# Nạp firmware vào ESP32
idf.py -p COMx flash

# Mở Serial Monitor để quan sát log hệ thống
idf.py -p COMx monitor
```

### Python Test Client (PC)
```bash
# Cài đặt thư viện phụ thuộc
pip install bleak

# Chạy script kiểm thử BLE tương tác
python tools/agv_ble_controller.py

# Hoặc chạy script chỉ định địa chỉ MAC/UUID (nếu cần)
python tools/agv_ble_controller.py --address "XX:XX:XX:XX:XX:XX"
```

---

## 4. Project Structure (Cấu Trúc Thư Mục)

```
agv_esp32/
├── CMakeLists.txt
├── sdkconfig.defaults
├── components/
│   ├── comms_protocol/             # Component chứa giao thức C thuần chia sẻ với STM32
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── comms_protocol.h
│   │   └── comms_protocol.c
│   ├── uart_service/               # Quản lý driver UART2, task nhận byte, giải mã frame nhị phân
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── uart_service.h
│   │   └── uart_service.c
│   ├── ble_nus/                    # Dịch vụ BLE Nordic UART Service (NimBLE)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── ble_nus.h
│   │   └── ble_nus.c
│   └── button/                     # Quản lý nút bấm BOOT (GPIO0)
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── button.h
│       └── button.c
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                     # Khởi tạo hệ thống, bridge điều phối BLE <-> UART
│   ├── ble_cmd_dispatcher.h       # Khai báo bộ phân tích cú pháp lệnh text BLE
│   └── ble_cmd_dispatcher.c       # Xử lý lệnh text BLE, đóng gói UART & định dạng chuỗi trả về
├── tools/
│   └── agv_ble_controller.py       # Script Python kết nối BLE và giao diện CLI tương tác
└── docs/
    ├── specs/
    │   └── SPEC-agv-esp32-binary-gateway.md  # Tài liệu đặc tả kỹ thuật này
    ├── agv_system_overview_for_esp32.md
    ├── esp32_uart_protocol_guide.md
    └── ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md
```

---

## 5. Protocol & Interface Specification

### 5.1 Giao Thức Nhị Phân UART (ESP32 <-> STM32)
Theo chuẩn `comms_protocol`:
- Header: `[0xAA 0x55] [MSG_ID] [LENGTH]`
- Payload: `[PAYLOAD_BYTES (0..128)]`
- Checksum: `[CRC16_H] [CRC16_L]` (CRC16-CCITT Poly 0x1021, Init 0xFFFF tính trên `MSG_ID + LENGTH + PAYLOAD`).

### 5.2 Bảng Lệnh BLE Cấp Cao (Python Client <-> ESP32)

| Lệnh Text BLE (gửi từ Python) | Tham Số | Gói Nhị Phân UART Sinh Ra | Mô Tả Hành Vi | Phản Hồi Trả Về Python |
|---|---|---|---|---|
| `PING` | Không | `AGV_MSG_PING` (0x01) | Kiểm tra kết nối 2 MCU | `[RESP] PONG (STM32 Alive)` |
| `STOP` hoặc `EMG` | Không | `AGV_MSG_EMERGENCY_STOP` (0x02) | Phanh dừng khẩn cấp lập tức | `[RESP] EMERGENCY STOP SENT` |
| `STEP_TEST <motor> <pwm> <ms>` | `motor`: L/R hoặc 0/1<br>`pwm`: -1000..1000<br>`ms`: thời gian (ms) | `AGV_MSG_DIAG_STEP_TEST` (0x20) | Chạy thử 1 động cơ với PWM ấn định | `[RESP] STEP_TEST queued` kèm ACK từ STM32 |
| `SET_SPEED <left_pwm> <right_pwm>` | `left_pwm`: -1000..1000<br>`right_pwm`: -1000..1000 | `AGV_MSG_DIAG_SET_SPEED` (0x21) | Đặt mức PWM trực tiếp 2 bánh | `[RESP] SPEED set` |
| `SET_RPM <left_rpm> <right_rpm>` | Số thực RPM (ví dụ: `80.0 80.0`) | `AGV_MSG_DIAG_SET_RPM` (0x22) | Đặt vận tốc vòng kín RPM | `[RESP] RPM set` |
| `GET_ENC` | Không | `AGV_MSG_DIAG_GET_ENC` (0x23) | Đọc xung encoder 2 bánh | `[ENCODER] L: <val> \| R: <val>` |
| `RESET_ENC` | Không | `AGV_MSG_DIAG_RESET_ENC` (0x24) | Đặt lại xung encoder về 0 | `[RESP] ENCODER RESET OK` |
| `LINE_CALIB <mode>` | `mode`: 1 (Calib) | `AGV_MSG_DIAG_LINE_TEST` (0x26) | Hiệu chuẩn mảng quang trở | `[RESP] LINE CALIB started` |
| `ROUTE_LOAD <rpm> <actions...>` | `rpm`: ví dụ `80.0`<br>`actions`: chuỗi số hoặc chữ (`F,R,F,S` hoặc `1,3,1,0`) | `AGV_MSG_ROUTE_LOAD` (0x10) | Nạp chuỗi hành vi qua ngã rẽ | `[RESP] ROUTE LOADED (N steps)` |
| `ROUTE_START` | Không | `AGV_MSG_ROUTE_START` (0x11) | Bắt đầu chạy lộ trình | `[RESP] ROUTE STARTED` |
| `ROUTE_CANCEL` | Không | `AGV_MSG_ROUTE_CANCEL` (0x12) | Hủy lộ trình đang chạy | `[RESP] ROUTE CANCELLED` |
| `ROUTE_TEST` | Không | Gửi liên tiếp `ROUTE_LOAD` mẫu và `ROUTE_START` | Chạy nhanh bài test lộ trình 4 bước | `[RESP] ROUTE_TEST initialized` |
| `HELP` | Không | Không | Trợ giúp danh sách lệnh | In bảng danh sách lệnh hỗ trợ |

### 5.3 Định Dạng Log & Sự Kiện Tự Động Từ STM32 -> BLE
- **Gói `AGV_MSG_LOG` (0x99)**: Định dạng chuỗi: `[STM32 LOG] <nội dung log ASCII>\r\n`.
- **Gói `AGV_MSG_MISSION_STATUS` (0x90)**: Định dạng: `[MISSION] State: <IDLE|RUNNING|COMPLETED|ERROR_LOST>, Step: <idx>\r\n`.
- **Gói `AGV_MSG_ACK` (0x80)**: Định dạng: `[ACK] MsgID: 0x%02X, Status: %s\r\n`.

---

## 6. Code Style (Quy Chuẩn Viết Code)

### ESP32 (C Code)
- Tuân thủ tiêu chuẩn ESP-IDF C: sử dụng tiền tố `esp_err_t`, macro log `ESP_LOGI(TAG, ...)`, `ESP_LOGW`, `ESP_LOGE`.
- Không cấp phát động (`malloc`/`free`) trong vòng lặp xử lý byte UART để tránh phân mảnh bộ nhớ và rò rỉ RAM.
- Tận dụng bộ đệm tĩnh (Static Buffer) và Ringbuffer có sẵn của ESP-IDF UART driver.
- Ví dụ xử lý callback gói tin UART:
```c
static void on_stm32_frame_received(const agv_frame_t *frame, void *user_data) {
    switch (frame->msg_id) {
        case AGV_MSG_ACK: {
            const agv_payload_ack_t *ack = (const agv_payload_ack_t *)frame->payload;
            ESP_LOGI(TAG, "STM32 ACK msg_id=0x%02X status=%d", ack->acked_msg_id, ack->status_code);
            char buf[64];
            snprintf(buf, sizeof(buf), "[ACK] Msg:0x%02X Status:%d\r\n", ack->acked_msg_id, ack->status_code);
            ble_nus_send_str(buf);
            break;
        }
        case AGV_MSG_LOG: {
            char log_msg[AGV_PROTOCOL_MAX_PAYLOAD + 1];
            uint8_t len = (frame->length < AGV_PROTOCOL_MAX_PAYLOAD) ? frame->length : AGV_PROTOCOL_MAX_PAYLOAD;
            memcpy(log_msg, frame->payload, len);
            log_msg[len] = '\0';
            ESP_LOGI(TAG, "[STM32] %s", log_msg);
            char ble_buf[AGV_PROTOCOL_MAX_PAYLOAD + 16];
            snprintf(ble_buf, sizeof(ble_buf), "[STM32 LOG] %s\r\n", log_msg);
            ble_nus_send_str(ble_buf);
            break;
        }
        default:
            break;
    }
}
```

### Python Code Style
- Sử dụng chuẩn PEP8, type hints, thư viện `asyncio` chuẩn và `bleak.BleakClient`.
- Bắt lỗi mất kết nối BLE (`disconnected_callback`) và tự động thông báo rõ ràng.
- Giao diện console phân biệt màu sắc hoặc prefix rõ ràng (`[SEND]`, `[RECV]`, `[STM32 LOG]`).

---

## 7. Testing Strategy (Chiến Lược Kiểm Thử)

1. **Kiểm thử đơn vị tầng giao thức (`comms-protocol`)**:
   - Kiểm tra tính đúng đắn của hàm `agv_crc16_ccitt()` với các chuỗi mẫu.
   - Kiểm tra `agv_parser_feed_byte()` đối với các frame bị ngắt đoạn hoặc frame có byte rác phía trước Header.
2. **Kiểm thử tích hợp UART Loopback (Hardware-in-the-loop / Bench Test)**:
   - Nối chân GPIO16 (RX) sang GPIO17 (TX) trên ESP32 để tự gửi tự nhận frame nhị phân, kiểm tra độ ổn định ở 115200 bps.
3. **Kiểm thử kết nối với STM32**:
   - Nối dây chéo TX-RX và chung GND với STM32F103.
   - Bấm nút BOOT trên ESP32 -> gửi PING -> quan sát LED PC13 trên STM32 nháy và nhận PONG.
4. **Kiểm thử toàn diện từ PC qua BLE Client**:
   - Khởi chạy `python tools/agv_ble_controller.py`.
   - Kết nối BLE tới thiết bị `AGV_ESP32`.
   - Chạy tuần tự các lệnh: `PING`, `STEP_TEST 0 400 1000`, `GET_ENC`, `ROUTE_TEST`, `STOP`.
   - Kiểm tra tốc độ phản hồi và độ trễ nhận log qua BLE.

---

## 8. Boundaries (Ranh Giới Phát Triển)

- **Always (Luôn luôn thực hiện)**:
  - Kiểm tra tính toàn vẹn CRC16 trên tất cả các khung truyền UART từ STM32 trước khi phân tích payload.
  - Sử dụng chung mass (Common GND) giữa ESP32 và STM32 khi kết nối cáp vật lý.
  - Xử lý bất đồng bộ, không gọi `vTaskDelay` blocking bên trong callback của BLE hay UART interrupt.
- **Ask first (Hỏi trước khi thay đổi)**:
  - Thay đổi cấu hình chân phần cứng UART (GPIO16/17) hoặc baudrate 115200.
  - Thay đổi UUID chuẩn của Nordic UART Service (NUS).
  - Bổ sung hoặc sửa đổi các struct trong `comms_protocol.h`.
- **Never (Tuyệt đối không làm)**:
  - Không gửi chuỗi văn bản thuần (ASCII) trực tiếp xuống UART STM32 (gây lỗi giao thức nhị phân).
  - Không bỏ qua lỗi CRC16 một cách âm thầm mà không log cảnh báo.
  - Không phân bổ động (`malloc`) trong FreeRTOS ISR hoặc task chu kỳ cao.

---

## 9. Success Criteria (Tiêu Chí Nghiệm Thu)

- [ ] Dự án ESP-IDF biên dịch thành công không cảnh báo lỗi (`idf.py build`).
- [ ] Thành phần `comms_protocol` được tích hợp độc lập và dùng chung chuẩn xác với STM32.
- [ ] `uart_service` trên ESP32 tiếp nhận luồng byte, lọc bỏ byte rác và parse thành công các gói tin `AGV_MSG_ACK`, `AGV_MSG_PONG`, `AGV_MSG_LOG`, `AGV_MSG_MISSION_STATUS`, `AGV_MSG_DIAG_ENC_DATA`.
- [ ] Khi bấm nút BOOT (GPIO0), ESP32 phát gói tin nhị phân `AGV_MSG_PING` xuống STM32.
- [ ] Script Python `agv_ble_controller.py` quét thấy thiết bị BLE `AGV_ESP32`, tự động kết nối và mở giao diện dòng lệnh tương tác.
- [ ] Gửi lệnh `PING` từ Python qua BLE -> STM32 nhận và nháy LED -> Python in ra `[RESP] PONG (STM32 Alive)`.
- [ ] Gửi lệnh `STEP_TEST` từ Python -> Động cơ STM32 quay đúng thời gian và in log về Python.
- [ ] Gửi lệnh `STOP` từ Python -> STM32 lập tức phanh dừng động cơ.
- [ ] Toàn bộ log phát ra từ STM32 (`AGV_MSG_LOG`) hiển thị theo thời gian thực trên màn hình Python Client.

---

## 10. Open Questions (Câu Hỏi Mở)

Hiện tại không còn câu hỏi chặn (blocker). Nếu bạn cần bổ sung thêm bất kỳ lệnh test chuyên biệt nào khác (như tinh chỉnh thông số PID `PI_TUNE`), chúng ta có thể dễ dàng map thêm vào bảng lệnh của `ble_cmd_dispatcher`.
