# Task List: Giao Tiếp UART Nhị Phân STM32 - ESP32 & Gỡ Bỏ RTT

## Phase 1: Binary Protocol Engine & Shared C Library

### Task 1: Thiết Kế & Xây Dựng Bộ Giao Thức Nhị Phân C Thuần (`comms_protocol`)
**Description:** Xây dựng module C thuần (Zero-HAL, không malloc) bao gồm cấu trúc khung truyền (`SOF 0xAA 0x55`, `MSG_ID`, `LEN`, `PAYLOAD`, `CRC16`), giải thuật CRC16-CCITT, máy trạng thái giải mã dòng byte (`agv_protocol_feed_byte`) và các hàm đóng gói gói tin (`agv_protocol_pack_*`). File header có thể dùng chung 100% cho cả STM32 và ESP32.
**Acceptance criteria:**
- [x] Cung cấp enum `agv_msg_id_t` đầy đủ cho Route (`ROUTE_LOAD`, `ROUTE_START`, `ROUTE_CANCEL`), Diag (`STEP_TEST`, `SET_SPEED`, `SET_RPM`, `GET_ENC`, `RESET_ENC`, `PI_TUNE`, `LINE_TEST`), System (`PING`, `ACK`, `LOG`, `MISSION_STATUS`).
- [x] Hàm `agv_protocol_feed_byte()` xử lý luồng byte streaming, phát hiện đúng SOF, kiểm tra độ dài và đối chiếu CRC16.
- [x] Các hàm pack đóng gói chuẩn little-endian, không phụ thuộc kiến trúc vi điều khiển.
**Verification:**
- [x] Tests pass: Chạy unit test parser qua Python script mô phỏng (`tests/test_binary_protocol.py`).
- [x] Build succeeds: `cmake --build build/Debug` không có lỗi cú pháp.
**Dependencies:** None
**Files touched:**
- `Services/Inc/comms_protocol.h`
- `Services/Src/comms_protocol.c`

---

### Task 2: Xây Dựng Bộ Test Tự Động Cho Giao Thức Nhị Phân (`test_binary_protocol.py`)
**Description:** Viết kịch bản kiểm thử toàn diện cho giao thức nhị phân bằng Python: kiểm thử pack/unpack từng loại thông điệp, kiểm thử byte rác, frame cắt vụn (fragmented stream), sai lệch CRC16, và tràn buffer payload.
**Acceptance criteria:**
- [x] Kiểm thử thành công 100% các lệnh: PING, SET_SPEED, STEP_TEST, PI_TUNE, LINE_TEST, ROUTE_LOAD, LOG.
- [x] Xác minh parser phục hồi ngay lập tức khi gặp byte rác hoặc CRC sai mà không bị deadlock.
**Verification:**
- [x] Tests pass: `py -3 -m unittest tests/test_binary_protocol.py` đạt 100% pass (9/9 tests).
**Dependencies:** Task 1
**Files touched:**
- `tests/test_binary_protocol.py`

---

### Checkpoint 1: Sau Task 1 & 2
- [x] Giao thức nhị phân hoàn thiện, độc lập phần cứng (Zero-HAL)
- [x] Bộ test unit test Python pass 100%
- [x] Sẵn sàng tích hợp vào driver phần cứng UART và hệ thống log

---

## Phase 2: UART Hardware Binding & SEGGER RTT Removal

### Task 3: Gỡ Bỏ SEGGER RTT & Chuyển Hệ Thống Log Sang Gói Tin Nhị Phân Qua UART DMA
**Description:** Loại bỏ hoàn toàn mã nguồn và header `SEGGER_RTT` khỏi dự án. Thay thế các lệnh in log cũ bằng macro `AGV_LOG()` đóng gói chuỗi vào frame `MSG_LOG` (ID `0x99`) gửi qua UART1 DMA. Cấu hình `AGV_ENABLE_UART_TTL = 1` trong `app_config.h` và khởi tạo `BSP_UART_Init(&huart1)` trong `App_Init()`.
**Acceptance criteria:**
- [x] `CMakeLists.txt` không còn liên kết tới `Drivers/SEGGER_RTT`.
- [x] Không còn bất kỳ file nào trong `Application/`, `Diagnostics/`, `BSP/` `#include "SEGGER_RTT.h"`.
- [x] `app_config.h` kích hoạt `AGV_ENABLE_UART_TTL 1`.
- [x] `App_Init()` gọi `BSP_UART_Init(&huart1)` để kích hoạt nhận DMA Circular và ngắt UART IDLE.
**Verification:**
- [x] Build succeeds: `cmake --build build/Debug` hoàn tất thành công 100%, không còn cảnh báo hoặc thiếu symbol RTT.
- [x] Grep kiểm tra: `grep -r "SEGGER_RTT" Application/ Diagnostics/ BSP/ Core/` trả về rỗng.
**Dependencies:** Task 1
**Files touched:**
- `CMakeLists.txt`
- `Application/Inc/app_config.h`
- `Application/Src/app_main.c`
- `Diagnostics/Src/diag_motor.c`
- `Diagnostics/Src/diag_ir.c`
- `Application/Src/app_motor_control.c`
- `BSP/Src/bsp_ir.c`

---

### Task 4: Cập Nhật Tài Liệu Cấu Hình Dự Án (`GEMINI.md` & Script)
**Description:** Cập nhật file chỉ dẫn `GEMINI.md` phản ánh việc chuyển đổi từ RTT sang UART1 TTL kết nối ESP32.
**Acceptance criteria:**
- [x] `GEMINI.md` loại bỏ hướng dẫn OpenOCD RTT server port 9090, cập nhật baud rate UART 115200 và kết nối chân PA9/PA10.
- [x] Thư mục build được dọn sạch và rebuild từ đầu không lỗi.
**Verification:**
- [x] Build succeeds: `ninja -C build/Debug clean && cmake --build build/Debug`
**Dependencies:** Task 3
**Files touched:**
- `GEMINI.md`

---

### Checkpoint 2: Sau Task 3 & 4
- [x] Mã nguồn STM32 hoàn toàn sạch bóng thư viện SEGGER RTT
- [x] Firmware biên dịch thành công, UART DMA sẵn sàng truyền nhận 2 chiều
- [x] Hệ thống log debug tự động chuyển thành gói nhị phân gửi qua UART1

---

## Phase 3: Route Queue & Autonomous Crossline Navigation

### Task 5: Nâng Cấp `app_line_follow` Hỗ Trợ Chuỗi Lộ Trình Tự Hành (Route Queue FSM)
**Description:** Nâng cấp máy trạng thái bám line trong `app_line_follow` để lưu trữ chuỗi bước hành động `route_actions[]`. Khi phát hiện vạch ngang ngã rẽ:
- Thực hiện hành động tại bước hiện tại (`FORWARD`: chạy qua vạch; `TURN_LEFT`: quay trái 90 độ bắt lại line; `TURN_RIGHT`: quay phải 90 độ bắt lại line; `STOP`: phanh dừng xe).
- Tự động nhảy sang bước tiếp theo với cơ chế debounce chống quét lặp vạch ngang.
- Khi hoàn thành bước cuối cùng, dừng xe và kích hoạt callback báo hoàn thành lộ trình.
**Acceptance criteria:**
- [x] Định nghĩa `agv_route_action_t`: `ACTION_STOP = 0`, `ACTION_FORWARD = 1`, `ACTION_TURN_LEFT = 2`, `ACTION_TURN_RIGHT = 3`.
- [x] Hỗ trợ hàm `App_LineFollow_LoadRoute(const uint8_t *actions, uint8_t count, float target_rpm)`.
- [x] Xử lý ngã rẽ chính xác, không bị kích hoạt trùng nhiều lần tại một vạch ngang (debounce >= 300ms).
- [x] Phát tín hiệu sự kiện khi kết thúc lộ trình hoặc khi mất line.
**Verification:**
- [x] Tests pass: Bộ test mô phỏng FSM logic ngã rẽ trong Python pass.
- [x] Build succeeds: `cmake --build build/Debug` pass.
**Dependencies:** Task 1, Task 3
**Files touched:**
- `Application/Inc/app_line_follow.h`
- `Application/Src/app_line_follow.c`

---

### Task 6: Tích Hợp Xử Lý Gói Tin Nhị Phân Trong `app_comms`
**Description:** Viết lại `app_comms.c` để đọc stream dữ liệu từ `BSP_UART_ReadByte()`, đưa vào bộ phân giải `agv_protocol_feed_byte()`. Khi có frame hợp lệ:
- Điều phối các lệnh điều khiển lộ trình (`CMD_ROUTE_LOAD`, `CMD_ROUTE_START`, `CMD_ROUTE_CANCEL`, `CMD_EMERGENCY_STOP`).
- Phản hồi ACK nhị phân (`RESP_ACK`) và phát sự kiện trạng thái (`EVENT_MISSION_STATUS`).
- Cung cấp hàm gửi log nhị phân `App_Comms_SendLog(const char *text)`.
**Acceptance criteria:**
- [x] `App_Comms_Process()` đọc và parse toàn bộ byte sẵn có trong UART RX RingBuffer.
- [x] Lệnh `CMD_EMERGENCY_STOP` ngắt tức thời mọi chuyển động động cơ và reset lộ trình.
- [x] Trả về `RESP_ACK` kèm mã trạng thái OK hoặc ERROR.
**Verification:**
- [x] Build succeeds: `cmake --build build/Debug` pass.
**Dependencies:** Task 1, Task 5
**Files touched:**
- `Application/Inc/app_comms.h`
- `Application/Src/app_comms.c`

---

## Phase 4: Diagnostics Support & Verification

### Task 7: Tích Hợp Toàn Bộ Lệnh Chẩn Đoán & Kiểm Thử Phần Cứng Qua Binary Frame
**Description:** Kết nối các thông điệp chẩn đoán nhị phân vào các service thực thi tương ứng:
- `CMD_DIAG_STEP_TEST` -> gọi `Diag_Motor_Start()`
- `CMD_DIAG_LINE_TEST` -> gọi `Diag_IR_Start()`
- `CMD_DIAG_SET_SPEED` & `CMD_DIAG_SET_RPM` -> gọi `BSP_Motor_SetSpeeds()` / `App_MotorControl_SetTargetRPM()`
- `CMD_DIAG_GET_ENC` -> đọc `BSP_Encoder_GetCount()` và gửi frame `RESP_DIAG_ENC`
- `CMD_DIAG_RESET_ENC` -> gọi `BSP_Encoder_ResetAll()`
- `CMD_DIAG_PI_TUNE` -> gọi `App_MotorControl_SetGains()`
**Acceptance criteria:**
- [x] Tất cả các bài test phần cứng độc lập hoạt động trơn tru qua lệnh nhị phân mà không cần RTT.
- [x] Đọc được số xung encoder 2 bánh trả về trong frame nhị phân `RESP_DIAG_ENC`.
**Verification:**
- [x] Tests pass: Kiểm thử dispatching các lệnh chẩn đoán.
- [x] Build succeeds: `cmake --build build/Debug` pass.
**Dependencies:** Task 6
**Files touched:**
- `Application/Src/app_comms.c`
- `Diagnostics/Src/diag_motor.c`
- `Diagnostics/Src/diag_ir.c`

---

### Task 8: Kiểm Thử Toàn Diện Tích Hợp & End-to-End Simulation
**Description:** Viết test script Python mô phỏng máy chủ ESP32: đóng gói frame UART nhị phân, gửi lệnh nạp lộ trình FSM, kích hoạt các chế độ test bàn, xác minh frame ACK, frame trạng thái hoàn thành và frame log từ STM32.
**Acceptance criteria:**
- [x] Toàn bộ kịch bản giao tiếp giữa ESP32 giả lập và STM32 chạy pass 100%.
- [x] Cập nhật các file unit test cũ trong `tests/` đồng bộ theo giao thức mới.
**Verification:**
- [x] Tests pass: `py -3 -m unittest discover tests` hoàn tất không lỗi (110/110 tests OK).
**Dependencies:** Task 6, Task 7
**Files touched:**
- `tests/test_e2e_comms.py`
- `tests/test_app_comms_refactor.py`
- `tests/test_app_line_follow_integration.py`
- `tests/test_comms_protocol.py`

---

### Checkpoint 3: Sau Task 5-8
- [x] STM32 hoàn tất hỗ trợ đầy đủ các lệnh Lộ trình tự hành và Chẩn đoán phần cứng
- [x] Hệ thống kiểm thử tự động xác nhận tính toàn vẹn của firmware
- [x] Sẵn sàng bàn giao tài liệu kỹ thuật cho lập trình viên ESP32

---

## Phase 5: ESP32 Developer Documentation & Integration Guide

### Task 9: Viết Tài Liệu Chi Tiết Hướng Dẫn Phát Triển Firmware ESP32 (`docs/esp32_uart_protocol_guide.md`)
**Description:** Viết tài liệu hướng dẫn hoàn chỉnh, chi tiết để kỹ sư lập trình ESP32 (dùng ESP-IDF hoặc Arduino) tích hợp giao tiếp với STM32:
- Sơ đồ nối dây phần cứng (PA9 TX <-> RX, PA10 RX <-> TX, GND, mức logic 3.3V).
- Đặc tả khung truyền nhị phân (Frame structure, CRC16 C code, Bảng mã lệnh đầy đủ kèm struct payload).
- Biểu đồ trình tự FSM: Quy trình nạp lộ trình, bắt đầu chạy, nhận sự kiện đến đích, xử lý sự cố mất vạch và dừng khẩn cấp.
- Hướng dẫn dùng trực tiếp file `comms_protocol.h` / `.c` trên ESP32.
- Mã nguồn mẫu (Ready-to-use C/C++ snippet) cho ESP32 khởi tạo UART, gửi lộ trình và parse frame phản hồi.
**Acceptance criteria:**
- [x] Tài liệu rõ ràng, dễ hiểu, đầy đủ bảng tra cứu từng byte của mọi gói tin.
- [x] Có mã nguồn ví dụ mẫu chạy được trên Arduino ESP32 / PlatformIO.
**Verification:**
- [x] Kiểm tra tài liệu markdown định dạng chuẩn, đầy đủ bảng tra cứu và diagram.
**Dependencies:** Task 1, Task 6, Task 7
**Files touched:**
- `docs/esp32_uart_protocol_guide.md`
