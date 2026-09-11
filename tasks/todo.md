# Task List: AGV ESP32 Binary Gateway & Wireless BLE Diagnostics

## Phase 1: Foundation & Binary UART Service

### Task 1: Thiết Lập & Kiểm Tra Component `comms_protocol` Trong `agv_esp32`
**Description:** Đảm bảo component `components/comms_protocol` được đăng ký đúng chuẩn CMake của ESP-IDF v6.x, cung cấp đầy đủ các struct packed, enum mã lệnh và hàm pack/parse C thuần dùng chung với STM32.
**Acceptance criteria:**
- [x] File `components/comms_protocol/CMakeLists.txt` đăng ký thư viện với thư mục `include/` và mã nguồn `comms_protocol.c`.
- [x] Component không phụ thuộc bất kỳ thư viện HAL vi điều khiển nào (Zero-HAL, pure C).
- [x] Có thể `#include "comms_protocol.h"` từ bất kỳ component nào trong `agv_esp32`.
**Verification:**
- [x] Build succeeds: `idf.py reconfigure` nhận diện thành công component `comms_protocol`.
**Dependencies:** None
**Files touched:**
- `agv_esp32/components/comms_protocol/CMakeLists.txt`
- `agv_esp32/components/comms_protocol/include/comms_protocol.h`
- `agv_esp32/components/comms_protocol/comms_protocol.c`
**Estimated scope:** Small (1 file)

---

### Task 2: Tái Cấu Trúc `uart_service` Thành Bộ Thu Phát Khung Truyền Nhị Phân Zero-Malloc
**Description:** Loại bỏ cơ chế phát hiện pattern chuỗi ký tự ASCII (`\r\n`) trong `uart_service`. Thay thế bằng FreeRTOS Task `uart_rx_task` đọc luồng byte từ UART2 (`GPIO16/17` @ 115200) và nạp vào máy trạng thái `agv_parser_feed_byte()`. Bổ sung API gửi frame nhị phân `uart_service_send_frame()`, `uart_service_send_msg()` và cơ chế đăng ký callback `uart_service_register_frame_cb()`.
**Acceptance criteria:**
- [x] Header `uart_service.h` định nghĩa kiểu callback `agv_frame_rx_cb_t` và cấu trúc cấu hình `uart_service_config_t`.
- [x] `uart_rx_task` cấp phát tĩnh toàn bộ buffer giải mã, tuyệt đối không gọi `malloc`/`free` trong luồng nhận byte.
- [x] Khi FSM phát hiện frame hoàn chỉnh hợp lệ CRC16, gọi callback chuyển giao dữ liệu tức thời.
- [x] Hàm `uart_service_send_msg()` đóng gói đúng Header `0xAA 0x55`, ID, Length, Payload và CRC16 trước khi gửi qua `uart_write_bytes()`.
**Verification:**
- [x] Build succeeds: `idf.py build` biên dịch component `uart_service` không có cảnh báo hoặc lỗi cú pháp.
**Dependencies:** Task 1
**Files touched:**
- `agv_esp32/components/uart_service/include/uart_service.h`
- `agv_esp32/components/uart_service/uart_service.c`
- `agv_esp32/components/uart_service/CMakeLists.txt`
**Estimated scope:** Medium (3 files)

---

### Checkpoint 1: Sau Phase 1
- [x] `comms_protocol` và `uart_service` biên dịch sạch sẽ trên ESP-IDF v6.x.
- [x] Tầng truyền thông nhị phân phần cứng sẵn sàng tiếp nhận và điều phối lệnh.

---

## Phase 2: BLE Command Bridge & Dispatcher

### Task 3: Xây Dựng Module `ble_cmd_dispatcher` (Text CLI -> Binary UART & Response Formatter)
**Description:** Tạo module `ble_cmd_dispatcher.c/.h` trong `main/` đảm nhiệm vai trò cầu nối 2 chiều:
1. Chiều Xuống (BLE -> STM32): Nhận chuỗi text từ BLE NUS, phân tích cú pháp và chuyển đổi thành gói tin nhị phân tương ứng:
   - `PING` $\rightarrow$ `AGV_MSG_PING` (0x01)
   - `STOP` / `EMG` $\rightarrow$ `AGV_MSG_EMERGENCY_STOP` (0x02)
   - `STEP_TEST <motor> <pwm> <ms>` $\rightarrow$ `AGV_MSG_DIAG_STEP_TEST` (0x20)
   - `SET_SPEED <left> <right>` $\rightarrow$ `AGV_MSG_DIAG_SET_SPEED` (0x21)
   - `SET_RPM <left> <right>` $\rightarrow$ `AGV_MSG_DIAG_SET_RPM` (0x22)
   - `GET_ENC` $\rightarrow$ `AGV_MSG_DIAG_GET_ENC` (0x23)
   - `RESET_ENC` $\rightarrow$ `AGV_MSG_DIAG_RESET_ENC` (0x24)
   - `PI_TUNE <kp> <ki> <kff>` $\rightarrow$ `AGV_MSG_DIAG_PI_TUNE` (0x25)
   - `MONITOR_IR [ms]` $\rightarrow$ `AGV_MSG_DIAG_LINE_TEST` (0x26, mode=0, rpm=0)
   - `LINE_CALIB [ms]` $\rightarrow$ `AGV_MSG_DIAG_LINE_TEST` (0x26, mode=1, rpm=0)
   - `LINE_TEST <mode> <ms> <rpm>` $\rightarrow$ `AGV_MSG_DIAG_LINE_TEST` (0x26)
   - `ROUTE_LOAD <rpm> <actions>` $\rightarrow$ `AGV_MSG_ROUTE_LOAD` (0x10)
   - `ROUTE_START` $\rightarrow$ `AGV_MSG_ROUTE_START` (0x11)
   - `ROUTE_CANCEL` $\rightarrow$ `AGV_MSG_ROUTE_CANCEL` (0x12)
   - `ROUTE_TEST` $\rightarrow$ Chuỗi mẫu `ROUTE_LOAD` + `ROUTE_START`
   - `HELP` $\rightarrow$ In bảng hướng dẫn cú pháp
2. Chiều Lên (STM32 -> BLE): Nhận frame nhị phân từ callback của `uart_service`, định dạng thành chuỗi ký tự có tiền tố rõ ràng (`[ACK]`, `[RESP]`, `[STM32 LOG]`, `[MISSION]`, `[ENCODER]`, `DATA:...`, `DATA:LINE,...`) và gửi về client qua `ble_nus_send_str()`.
**Acceptance criteria:**
- [x] Phân tích chính xác tất cả 15 lệnh text trên, có kiểm tra lỗi cú pháp và kiểm tra giá trị tham số hợp lệ.
- [x] Định dạng đầy đủ các thông điệp phản hồi từ STM32, không làm mất mát ký tự.
- [x] Các chuỗi telemetry bắt đầu bằng `DATA:` được truyền nguyên bản để client PC dễ dàng parse regex.
**Verification:**
- [x] Build succeeds: `idf.py build` hoàn tất không lỗi.
**Dependencies:** Task 2
**Files touched:**
- `agv_esp32/main/ble_cmd_dispatcher.h`
- `agv_esp32/main/ble_cmd_dispatcher.c`
- `agv_esp32/main/CMakeLists.txt`
**Estimated scope:** Medium (3 files)

---

### Task 4: Cập Nhật `main.c`, Kích Hoạt MTU 256 Cho `ble_nus` & Gán Nút BOOT
**Description:** Sửa lỗi cú pháp trong `main.c`, kết nối toàn bộ các thành phần hệ thống:
- Cấu hình MTU kích thước lớn $\ge 256$ byte trong `sdkconfig.defaults` và `ble_nus.c` để truyền tải telemetry tốc độ cao mà không ngắt gói.
- Đăng ký `ble_cmd_dispatcher_handle_text` làm callback nhận dữ liệu của BLE NUS.
- Đăng ký `ble_cmd_dispatcher_handle_frame` làm callback nhận frame của UART Service.
- Cấu hình nút BOOT (GPIO0): khi ấn nút gửi frame `AGV_MSG_PING` xuống STM32 và in thông báo lên BLE.
**Acceptance criteria:**
- [x] Loại bỏ hoàn toàn các lỗi cú pháp cũ trong `main.c`.
- [x] ESP32 khởi động sạch sẽ: NVS flash, NimBLE stack, UART2 driver, BOOT button handler.
- [x] Cấu hình MTU 256 bytes hoạt động với client BLE.
**Verification:**
- [x] Build succeeds: `idf.py build` hoàn tất 100% không cảnh báo lỗi trên ESP-IDF v6.x (`v6.0.2`).
**Dependencies:** Task 3
**Files touched:**
- `agv_esp32/main/main.c`
- `agv_esp32/components/ble_nus/ble_nus.c`
- `agv_esp32/sdkconfig.defaults`
**Estimated scope:** Small (3 files)

---

### Checkpoint 2: Sau Phase 2
- [x] Firmware `agv_esp32` đã sẵn sàng 100% mã nguồn, biên dịch thành công nhị phân `.bin`.
- [x] Đầy đủ các kênh BLE NUS <-> Binary UART2 DMA và xử lý nút bấm vật lý.

---

## Phase 3: Python BLE Wireless Client & Diagnostics Tool

### Task 5: Xây Dựng Lõi Client BLE Tương Tác (`agv_ble_controller.py`)
**Description:** Xây dựng script Python PC sử dụng thư viện `bleak` và `asyncio`:
- Tự động quét BLE tìm thiết bị `AGV_ESP32` (hoặc Service UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`).
- Kết nối, thương lượng MTU và đăng ký nhận thông báo (Notification) trên characteristic TX.
- Cung cấp giao diện dòng lệnh tương tác REPL prompt (hỗ trợ lịch sử lệnh, thoát bằng `quit`/`exit`).
- In màu các dòng log theo tiền tố (`[ACK]`, `[STM32 LOG]`, `[MISSION]`, `[ENCODER]`).
**Acceptance criteria:**
- [ ] Script tự động tìm và kết nối nhanh tới ESP32 qua BLE.
- [ ] Nhập lệnh `ping`, `stop`, `get_enc`, `reset_enc`, `set_rpm`, `route_test` hoạt động mượt mà.
- [ ] Xử lý ngắt kết nối an toàn khi người dùng bấm Ctrl+C hoặc thoát chương trình.
**Verification:**
- [ ] Syntax check: `python -m py_compile agv_esp32/tools/agv_ble_controller.py`.
**Dependencies:** Task 4
**Files touched:**
- `agv_esp32/tools/agv_ble_controller.py`
**Estimated scope:** Medium (1 file)

---

### Task 6: Tích Hợp Các Tính Năng Chẩn Đoán Không Dây Vào Client (`monitor_ir`, `calibrate`, `plot_step`)
**Description:** Mở rộng `agv_ble_controller.py` với 3 chế độ chẩn đoán chuyên sâu:
1. `monitor_ir [duration]`:
   - Gửi `MONITOR_IR <ms>` qua BLE.
   - Bắt dòng dữ liệu `DATA:LINE,<ms>,<raw0..4>,<norm0..4>,<err>,<seen>`.
   - Xóa màn hình console và render thanh đo đồ họa Unicode `|██████░░░░|` cho cả 5 mắt cảm biến ở tần số 20Hz, hiển thị sai số $e_{line}$ và trạng thái bắt vạch.
2. `calibrate [duration]`:
   - Gửi `LINE_CALIB <ms>` qua BLE.
   - Theo dõi running min/max raw ADC khi người dùng lia giấy trắng và vạch đen.
   - Kiểm tra dải động $\Delta_i \ge 100$, in bảng tổng kết và cho phép xuất file `app_line_calib_defaults.h` / `line_calib.json`.
3. `step <motor> <pwm> <ms> [--plot]`:
   - Gửi `STEP_TEST <motor> <pwm> <ms>` qua BLE.
   - Thu thập dòng `DATA:<ms>,...,<rpm>`, lưu vào file CSV `step_response_ble.csv`.
   - Nếu có cờ `--plot`, mở cửa sổ đồ thị Matplotlib hiển thị đường cong gia tốc, vận tốc đỉnh và vận tốc xác lập.
**Acceptance criteria:**
- [ ] Chạy trơn tru cả 3 tính năng qua CLI flags (`--monitor-ir`, `--calibrate`, `--step`) lẫn bên trong REPL shell.
- [ ] Bắt lỗi timeout hoặc mất kết nối giữa chừng mà không bị crash.
**Verification:**
- [ ] Syntax & dry-run test: Chạy script với `--help` hiển thị đầy đủ các tùy chọn.
**Dependencies:** Task 5
**Files touched:**
- `agv_esp32/tools/agv_ble_controller.py`
**Estimated scope:** Medium (1 file)

---

### Task 7: Viết Bộ Test Tự Động Kiểm Thử Client Parser & Protocol Logic (`test_ble_controller.py`)
**Description:** Viết bộ unit test tự động bằng `unittest` trong Python nhằm kiểm chứng:
- Thuật toán trích xuất regex cho `DATA:LINE` và `DATA:STEP`.
- Logic tích lũy min/max và kiểm tra dải động quang học $\Delta \ge 100$ trong calibration.
- Bộ tokenizer phân tích lệnh text của dispatcher.
- Đảm bảo bộ công cụ hoạt động ổn định và tin cậy trước khi nạp vào mạch thực tế.
**Acceptance criteria:**
- [ ] Toàn bộ các test case về parsing dòng dữ liệu, kiểm tra ngưỡng calibration và phân tích cú pháp lệnh pass 100%.
**Verification:**
- [ ] Tests pass: `py -3 -m unittest agv_esp32/tools/test_ble_controller.py`.
**Dependencies:** Task 6
**Files touched:**
- `agv_esp32/tools/test_ble_controller.py`
**Estimated scope:** Small (1 file)

---

### Checkpoint 3: Sau Phase 3
- [ ] Toàn bộ 7 task hoàn tất 100%.
- [ ] Firmware `agv_esp32` biên dịch thành công sạch sẽ trên ESP-IDF v6.x.
- [ ] Bộ công cụ Python client và unit test tự động pass 100%.
- [ ] Hệ thống sẵn sàng cho bước nạp mạch và kiểm thử thực địa không dây trên sa bàn!
