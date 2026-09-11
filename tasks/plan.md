# Implementation Plan: AGV ESP32 Binary Gateway & Wireless BLE Diagnostics

## 1. Overview
Dự án AGV bước vào giai đoạn hoàn thiện vi điều khiển thứ hai: **ESP32 Gateway & Coordinator (`agv_esp32`)** trên môi trường **ESP-IDF v6.x (v6.0.2)**. 
Hệ thống kết nối trực tiếp với **STM32F103** qua UART2 (GPIO16/17 @ 115200 8N1) bằng giao thức nhị phân chuẩn `comms_protocol` (ADR-003), đồng thời cung cấp cổng kết nối Bluetooth Low Energy (NimBLE NUS) và bộ công cụ chẩn đoán không dây trên máy tính (`agv_ble_controller.py`) phục vụ:
- Điều khiển lộ trình tự hành & dừng phanh khẩn cấp.
- Đọc cảm biến & kiểm tra động cơ.
- Chẩn đoán đáp ứng bước động cơ (`plot_step`).
- Giám sát trực quan 5 mắt IR thời gian thực (`monitor_ir`).
- Hiệu chuẩn quang học bàn test không dây (`calibrate_line`).

---

## 2. Architecture Decisions & Design Patterns

### 2.1 Tách Biệt Ranh Giới Giao Thức (Zero-HAL Protocol Shared Library)
- Sử dụng trực tiếp `comms_protocol.c` và `comms_protocol.h` (đặt trong `components/comms_protocol/`) dùng chung 100% với STM32.
- Bảo vệ dữ liệu bằng CRC16-CCITT (Poly `0x1021`, Init `0xFFFF`) trên tất cả các khung truyền MCU-to-MCU.

### 2.2 Luồng UART Nhị Phân Zero-Malloc (FreeRTOS RX Task)
- `uart_service` gỡ bỏ hoàn toàn logic cắt chuỗi ASCII CLI cũ (`\r\n`).
- Triển khai `uart_rx_task` tĩnh (Stack 4KB, Priority 10) đọc byte stream qua `uart_read_bytes()` nạp trực tiếp vào máy trạng thái `agv_parser_feed_byte()`.
- Tuyệt đối không dùng `malloc()` / `free()` trong chu trình streaming, loại trừ nguy cơ phân mảnh heap.
- Cung cấp hàm callback đăng ký nhận frame: `uart_service_register_frame_cb(agv_frame_rx_cb_t cb, void *user_data)`.

### 2.3 Cầu Nối Lệnh BLE Cấp Cao & Định Dạng Phản Hồi (`ble_cmd_dispatcher`)
- Nhận chuỗi văn bản UTF-8/ASCII từ BLE NUS RX Characteristic (`on_ble_data_received`).
- Phân tích cú pháp (tokenizer): `PING`, `STOP`, `STEP_TEST`, `MONITOR_IR`, `LINE_CALIB`, `LINE_TEST`, `GET_ENC`, `RESET_ENC`, `SET_SPEED`, `SET_RPM`, `PI_TUNE`, `ROUTE_LOAD`, `ROUTE_START`, `ROUTE_CANCEL`, `ROUTE_TEST`, `HELP`.
- Đóng gói frame nhị phân tương ứng và gửi sang STM32 qua `uart_service_send_msg()`.
- Đăng ký callback từ `uart_service`: định dạng các frame phản hồi từ STM32 (`ACK`, `PONG`, `LOG`, `DATA:LINE`, `DATA`, `MISSION_STATUS`, `ENC_DATA`) thành chuỗi text có tiền tố rõ ràng và bắn về client qua `ble_nus_send_str()`.

### 2.4 Tối Ưu Băng Thông & MTU Cho Telemetry Tốc Độ Cao
- Kích hoạt MTU thương lượng trên NimBLE NUS (`CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256`), cho phép một dòng telemetry `DATA:LINE,...` (khoảng 65 bytes) hoặc `DATA:...` được gửi trọn vẹn trong 1 packet notification duy nhất mà không bị cắt vụn.

### 2.5 Bộ Công Cụ Chẩn Đoán Không Dây Trên PC (`tools/agv_ble_controller.py`)
- Xây dựng trên nền thư viện `bleak` và `asyncio`.
- Hỗ trợ cả 2 chế độ:
  1. **Interactive REPL Prompt**: Nhập lệnh trực tiếp, xem log màu, phím tắt nhanh.
  2. **Standalone Sub-commands**:
     - `--monitor-ir [ms]`: Render thanh đo trực quan Unicode `|██████░░░░|` ở tần số 20Hz.
     - `--calibrate [ms]`: Quét min/max, kiểm tra $\Delta \ge 100$, xuất file `app_line_calib_defaults.h`.
     - `--step <motor> <pwm> <ms> [--plot]`: Ghi CSV và vẽ đồ thị Matplotlib.

---

## 3. Dependency Graph & Build Order

```
[comms_protocol] (Pure C Protocol Component)
       │
       ▼
[uart_service] (UART2 Driver & Binary Stream Parser Task)
       │
       ▼
[ble_cmd_dispatcher] & [main.c] (BLE NUS <-> Binary UART Bridge + BOOT Button)
       │
       ▼
[tools/agv_ble_controller.py] (Python Bleak Wireless Client & Diagnostics Suite)
```

---

## 4. Phase Breakdown & Verification Checkpoints

- **Phase 1: Foundation & Binary UART Service** (Task 1, 2)
  - Verify `comms_protocol` component.
  - Refactor `uart_service` sang bộ giải mã nhị phân stream byte Zero-malloc.
  - *Checkpoint 1*: Component build sạch sẽ.
- **Phase 2: BLE Command Bridge & Dispatcher** (Task 3, 4)
  - Xây dựng `ble_cmd_dispatcher` dịch 15 lệnh text sang binary frame và định dạng phản hồi.
  - Cập nhật `main.c`, kích hoạt MTU 256 trong `ble_nus`, gán nút BOOT GPIO0.
  - *Checkpoint 2*: Toàn bộ firmware `agv_esp32` biên dịch thành công 100% trên ESP-IDF v6.x.
- **Phase 3: Python BLE Wireless Client & Diagnostics Tool** (Task 5, 6, 7)
  - Triển khai `agv_ble_controller.py` với REPL prompt và streaming parser.
  - Tích hợp 3 module chẩn đoán: `monitor_ir`, `calibrate_line`, `plot_step`.
  - Viết bộ unit test tự động `test_ble_controller.py` kiểm thử toàn bộ parser/validator logic.
  - *Checkpoint 3*: Unit test pass 100%, sẵn sàng chạy thực tế.

---

## 5. Risks and Mitigations

| Rủi Ro | Mức Độ | Biện Pháp Giảm Thiểu |
|---|---|---|
| **Nghẽn hàng đợi BLE khi stream dữ liệu 20Hz** | Trung bình | Tăng MTU lên 256 byte, mỗi gói telemetry gửi trong 1 notification duy nhất; client dùng async queue không block UI. |
| **Byte rác UART do động cơ đảo chiều** | Trung bình | FSM `agv_parser_feed_byte` tự động đồng bộ lại Header `0xAA 0x55` và bỏ qua frame sai CRC16. |
| **Tràn bộ đệm Ringbuffer UART trên ESP32** | Thấp | Cấu hình kích thước RX Ringbuffer của ESP-IDF UART driver lên 2048 bytes. |
| **Lỗi tương thích API ESP-IDF v6.x** | Trung bình | Sử dụng các driver header mới (`driver/uart.h`, `driver/gpio.h`), cấu hình `sdkconfig.defaults` chuẩn xác. |
