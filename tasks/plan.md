# Implementation Plan: Giao Tiếp UART Nhị Phân STM32 - ESP32 & Gỡ Bỏ SEGGER RTT

## 1. Tổng Quan & Bối Cảnh
Dự án AGV chuyển từ giai đoạn kiểm thử bàn cắm dây nạp (ST-Link / SEGGER RTT) sang vận hành thực địa không dây trên sa bàn.
ESP32 đóng vai trò là BLE Gateway tiếp nhận lệnh lộ trình từ người dùng, chạy FSM điều phối cấp cao và truyền chuỗi hành động (`[THẲNG, RẼ_PHẢI, ...]`) xuống STM32 qua UART1.
STM32 đóng vai trò vi điều khiển thời gian thực phụ trách bám line, tự đếm vạch ngang IR để nhảy bước lộ trình tự hành, thực hiện các bài test/chẩn đoán phần cứng và gửi log/trạng thái sang ESP32 dưới dạng các gói tin nhị phân (Binary Frame).
Thư viện SEGGER RTT được gỡ bỏ hoàn toàn khỏi firmware STM32.

---

## 2. Quyết Định Kiến Trúc (Architecture Decisions)

### 2.1 Cấu Trúc Khung Truyền Nhị Phân (Binary Frame)
Khung truyền nhị phân chuẩn MCU-to-MCU gọn nhẹ, chống tràn bộ đệm và chống nhiễu UART:
```
+--------+--------+--------+------------+-------------------------+------------+------------+
| SOF1   | SOF2   | MSG_ID | LENGTH     | PAYLOAD (0..128 bytes)  | CRC16_H    | CRC16_L    |
| (0xAA) | (0x55) | (1 B)  | (1 B: 0..N)| (N Bytes)               | (1 B)      | (1 B)      |
+--------+--------+--------+------------+-------------------------+------------+------------+
```
- **SOF**: `0xAA 0x55` (2 bytes đồng bộ bắt đầu frame).
- **MSG_ID**: Mã định danh thông điệp (1 byte).
- **LENGTH**: Độ dài vùng Payload (1 byte, tối đa 128 bytes).
- **PAYLOAD**: Dữ liệu có cấu trúc nhị phân (struct packed, little-endian).
- **CRC16**: CRC16-CCITT (Polynomial `0x1021`, Seed `0xFFFF`) phủ toàn bộ từ `MSG_ID` đến hết `PAYLOAD`.

### 2.2 Danh Mục Thông Điệp (Message Catalog)
1. **Nhóm Điều Khiển Lộ Trình (Route / Mission Control)**:
   - `0x10` `CMD_ROUTE_LOAD`: Nạp chuỗi bước hành động `[num_steps] [target_rpm] [actions: FORWARD, TURN_LEFT, TURN_RIGHT, STOP]`.
   - `0x11` `CMD_ROUTE_START`: Kích hoạt xe bắt đầu chạy theo lộ trình đã nạp.
   - `0x12` `CMD_ROUTE_CANCEL` / `0x02` `CMD_EMERGENCY_STOP`: Phanh dừng khẩn cấp và hủy lộ trình.
   - `0x90` `EVENT_MISSION_STATUS`: STM32 báo trạng thái `[state: IDLE, RUNNING, COMPLETED, ERROR_LOST] [current_step]`.
2. **Nhóm Chẩn Đoán & Kiểm Thử Phần Cứng (Diagnostics & Test Mode)**:
   - `0x20` `CMD_DIAG_STEP_TEST`: Test đáp ứng bước động cơ (`motor_id`, `pwm`, `duration_ms`).
   - `0x21` `CMD_DIAG_SET_SPEED`: Đặt tốc độ PWM trực tiếp (`left`, `right`).
   - `0x22` `CMD_DIAG_SET_RPM`: Đặt vận tốc vòng kín PI (`left_rpm`, `right_rpm`).
   - `0x23` `CMD_DIAG_GET_ENC` & `0xA3` `RESP_DIAG_ENC`: Đọc số xung encoder 2 bánh (`enc_l`, `enc_r`).
   - `0x24` `CMD_DIAG_RESET_ENC`: Reset encoder về 0.
   - `0x25` `CMD_DIAG_PI_TUNE`: Cập nhật thông số PID (`Kp`, `Ki`, `Kff`).
   - `0x26` `CMD_DIAG_LINE_TEST`: Chạy bài test cảm biến IR (`mode: SENSOR, CALIB, TRACK, STOP`).
3. **Nhóm Hệ Thống & Giám Sát**:
   - `0x01` `CMD_PING` & `0x81` `RESP_PONG`: Kiểm tra kết nối 2 MCU.
   - `0x80` `RESP_ACK`: Phản hồi kết quả nhận lệnh (`acked_msg_id`, `status_code`).
   - `0x99` `MSG_LOG`: Đóng gói chuỗi log debug gửi sang ESP32.
   - `0x91` `MSG_TELEMETRY`: Định kỳ gửi tốc độ thực, góc Odometry và bitmask cảm biến IR.

### 2.3 Gỡ Bỏ SEGGER RTT & Chuyển Đổi Hệ Thống Log
- Loại bỏ thư viện `SEGGER_RTT` khỏi `CMakeLists.txt` và thư mục `Drivers/`.
- Macro `AGV_LOG(fmt, ...)` định dạng chuỗi vào bộ đệm tĩnh và đóng gói thành gói tin nhị phân `MSG_LOG` (ID `0x99`) gửi qua `BSP_UART_Write()` sang ESP32.
- Bật `AGV_ENABLE_UART_TTL 1` trong `app_config.h`, khởi tạo `BSP_UART_Init(&huart1)` trong `App_Init()`.

### 2.4 Quản Lý Chuỗi Hành Động Tự Hành (Autonomous Route Queue)
- STM32 lưu trữ một mảng tĩnh các bước hành động (`MAX_ROUTE_STEPS = 16`).
- Khi bám line, nếu mảng IR quét thấy vạch ngang:
  - Hành động hiện tại là `ACTION_FORWARD`: Bỏ qua/vượt qua ngã rẽ rồi tăng biến đếm bước.
  - Hành động hiện tại là `ACTION_TURN_LEFT`: Quay tại chỗ sang trái cho đến khi mắt IR giữa bắt lại line nhánh.
  - Hành động hiện tại là `ACTION_TURN_RIGHT`: Quay tại chỗ sang phải cho đến khi mắt IR giữa bắt lại line nhánh.
  - Hành động hiện tại là `ACTION_STOP` hoặc hết danh sách bước: Phanh dừng, chuyển trạng thái sang `COMPLETED`, phát gói `EVENT_MISSION_STATUS(COMPLETED)` lên ESP32.

---

## 3. Dependency Graph

```
[Services/comms_protocol] (Pure C Binary Protocol Parser & Packer)
         │
         ├── [BSP/bsp_uart] (UART1 DMA RX/TX Circular RingBuffer)
         │        │
         │        └── [Application/app_comms] (Command Dispatcher & Binary Frame Handler)
         │                 │
         │                 ├── [Application/app_line_follow] (Route Queue & Crossline Actions)
         │                 ├── [Application/app_motor_control] (Closed-loop Speed Execution)
         │                 └── [Diagnostics/diag_motor & diag_ir] (Bench Test Execution)
         │
         ├── [Gỡ Bỏ Drivers/SEGGER_RTT] (Thay bằng AGV_LOG binary frame)
         │
         └── [docs/esp32_uart_protocol_guide.md] (Tài liệu chi tiết & Sample Code cho ESP32)
```

---

## 4. Danh Sách Task Triển Khai
Chi tiết tại `tasks/todo.md`.
- **Phase 1**: Binary Protocol Engine & Shared C Library (Task 1, 2)
- **Phase 2**: UART DMA Binding & SEGGER RTT Removal (Task 3, 4)
- **Phase 3**: Route Queue & Autonomous Crossline Navigation (Task 5, 6)
- **Phase 4**: Diagnostics Support & End-to-End Verification (Task 7, 8)
- **Phase 5**: ESP32 Firmware Development Documentation (Task 9)

---

## 5. Rủi Ro & Giải Pháp Giảm Thiểu (Risks & Mitigations)
| Rủi Ro | Mức Độ | Giải Pháp |
|---|---|---|
| Nhiễu UART khi động cơ hoạt động | Trung bình | Khung truyền có CRC16-CCITT bảo vệ; bỏ qua frame hỏng mà không gây kẹt parser. |
| STM32 nhận diện nhầm vạch ngang nhiều lần liên tiếp | Cao | Cơ chế debounce thời gian (chặn nhận diện lại vạch ngang trong 300ms sau khi chuyển bước). |
| Tràn bộ đệm UART TX khi in log quá nhiều | Trung bình | Rate-limit cho log, gói log có độ dài tối đa 64 byte, hàng đợi TX RingBuffer 512 byte. |
| ESP32 mất kết nối giữa đường | Cao | Timeout Heartbeat an toàn: nếu không nhận được ping/keepalive từ ESP32 sau 3 giây, STM32 tự phanh dừng. |
