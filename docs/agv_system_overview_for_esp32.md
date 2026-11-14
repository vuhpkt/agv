# Tổng Quan Dự Án Xe Tự Hành AGV Dành Cho Lập Trình Viên ESP32

Tài liệu này được biên soạn dành riêng cho kỹ sư phát triển firmware trên **ESP32**, cung cấp cái nhìn toàn diện từ bức tranh tổng thể, kiến trúc phân tầng, nguyên lý chuyển động của xe AGV, cho đến hướng dẫn thiết kế máy trạng thái lộ trình (FSM) và giao tiếp với **STM32F103C8T6**.

---

## 1. Tổng Quan Dự Án

### 1.1 Mục Đích Thiết Kế Của Xe AGV
Đây là robot tự hành cỡ nhỏ dạng dẫn động vi sai hai bánh, trang bị mảng cảm biến quang học phía trước chuyên dùng để chạy bám vạch kẻ sàn trong nhà kho hoặc sa bàn mô phỏng:
- **Di chuyển**: Hai bánh chủ động độc lập điều khiển bằng động cơ DC giảm tốc kèm encoder hai kênh A/B, cùng bánh xe tự hướng trợ lực thăng bằng.
- **Dò đường**: Mảng cảm biến quang học hồng ngoại gắn sát mặt sàn phía trước đầu xe để liên tục đo đạc vị trí lệch so với tâm vạch dẫn hướng.
- **Sa bàn**: Vạch dẫn đường màu tương phản trên sàn. Tại các điểm giao nhau (ngã ba, ngã tư) hoặc điểm dừng trạm có bố trí vạch ngang cắt ngang đường đi.

### 1.2 Mục Tiêu Vận Hành Hệ Thống
1. Người vận hành sử dụng ứng dụng di động hoặc phần mềm quản lý kho kết nối không dây qua Bluetooth Low Energy (BLE) với ESP32.
2. Người dùng phát lệnh yêu cầu xe di chuyển giữa các trạm xác định trên sa bàn.
3. ESP32 tra cứu bản đồ sa bàn, chuyển yêu cầu thành chuỗi các bước di chuyển qua từng ngã rẽ.
4. ESP32 nạp chuỗi lệnh này xuống STM32 qua cổng UART bằng giao thức nhị phân.
5. STM32 tự động bám vạch, nhận diện vạch ngang để chuyển hướng và dừng hẳn khi đến đích, sau đó gửi thông báo trạng thái về ESP32.
6. ESP32 cập nhật trạng thái lên giao diện điều khiển của người dùng.

---

## 2. Kiến Trúc 2 Vi Điều Khiển Và Phân Chia Trách Nhiệm

Hệ thống tuân thủ nguyên lý phân tách trách nhiệm giữa bộ điều phối và bộ điều khiển chấp hành thời gian thực:

```
┌─────────────────────────────────────────────────────────────┐
│                ỨNG DỤNG ĐIỀU KHIỂN & GIAO DIỆN              │
└──────────────────────────────┬──────────────────────────────┘
                               │ Bluetooth Low Energy (BLE)
┌──────────────────────────────▼──────────────────────────────┐
│ [ESP32-S3] - BỘ ĐIỀU PHỐI VÀ TRUYỀN THÔNG                   │
│ 1. Gateway kết nối không dây BLE với người dùng.             │
│ 2. Quản lý bản đồ mạng lưới các trạm và nhánh rẽ.           │
│ 3. Lập kế hoạch đường đi và quản lý lộ trình.               │
│ 4. Chuyển tiếp log và dữ liệu chẩn đoán lên ứng dụng.        │
│ 5. Phát lệnh dừng khẩn cấp khi có sự cố.                    │
│ 6. Sẵn sàng tích hợp camera và thẻ nhớ.                     │
└──────────────────────────────┬──────────────────────────────┘
                               │ UART1 DMA 115200 8N1 (CRC16-CCITT)
                               │ ESP32-S3 GPIO42 (TX) <-> STM32 PA10 (RX)
                               │ ESP32-S3 GPIO41 (RX) <-> STM32 PA9 (TX)
┌──────────────────────────────▼──────────────────────────────┐
│ [STM32] - BỘ ĐIỀU KHIỂN CHUYỂN ĐỘNG THỜI GIAN THỰC          │
│ 1. Đọc ADC DMA 5 cảm biến quang học ở chu kỳ 10ms.          │
│ 2. Thuật toán bám vạch Line PID (chu kỳ 10ms).              │
│ 3. Đọc xung encoder và điều khiển vận tốc kín Motor PI.      │
│ 4. Hàng đợi lộ trình: Tự nhận diện vạch ngang và chuyển     │
│    bước, không cần chờ hỏi lại ESP32 tại từng ngã rẽ.       │
│ 5. Tự ngắt động cơ an toàn khi mất vạch liên tục quá 2.0s.   │
│ 6. Thực thi các bài kiểm tra phần cứng (Step Test, Diag).   │
└─────────────────────────────────────────────────────────────┘
```

### Thiết Kế Hàng Đợi Lộ Trình Độc Lập
- Khi xe đến ngã rẽ, STM32 không cần dừng lại gửi tin hỏi ESP32 để tránh xe bị giật cục và trễ quán tính tại từng giao lộ.
- ESP32 gửi toàn bộ danh sách bước ngay từ đầu qua gói tin `AGV_MSG_ROUTE_LOAD`. Trong suốt hành trình, STM32 tự động quét vạch ngang và chuyển bước độc lập. Khi hoàn thành bước cuối cùng, xe tự động phanh dừng và gửi gói tin thông báo trạng thái `EVENT_MISSION_STATUS(COMPLETED)` lên ESP32.

---

## 3. Các Thông Số Cơ Khí Và Chuyển Động Cần Lưu Ý

Lập trình viên cần lưu ý các thông số chuyển động và cơ học của xe để tính toán lộ trình và cấu hình vận tốc:

| Thông số | Cách xác định | Ý nghĩa đối với điều khiển |
|---|---|---|
| **Đường kính bánh xe ($D$)** | Đo đạc thực tế trên bánh xe AGV | Dùng để quy đổi từ số vòng quay sang quãng đường di chuyển: $S = \pi \times D \times \text{vòng}$. |
| **Khoảng cách 2 bánh ($L$)** | Đo đạc thực tế giữa 2 tâm bánh xe | Dùng để tính toán bán kính quay vòng vi sai và tốc độ góc khi rẽ. |
| **Độ phân giải Encoder ($CPR$)** | Xác định theo thông số động cơ và chế độ đọc Timer 4X | Số xung ghi nhận trên một vòng quay của bánh xe, phục vụ tính vận tốc góc. |
| **Vận tốc cơ sở khi bám line** | Cấu hình qua lệnh `ROUTE_LOAD` hoặc tham số mặc định | Vận tốc chạy thẳng trên sa bàn. Khi vào ngã rẽ, STM32 tự động giảm tốc để đảm bảo bám vạch an toàn. |
| **Dải điều khiển PWM TB6612** | `-1000` đến `+1000` | Điều chế độ rộng xung 20kHz, tương ứng $-100.0\%$ đến $+100.0\%$ công suất cấp cho động cơ. |
| **Thời gian ngắt an toàn khi mất vạch** | Cấu hình trong phần mềm (mặc định 2.0s) | Khi toàn bộ cảm biến mất vạch liên tục vượt ngưỡng này, STM32 tự dừng khẩn cấp và báo lỗi. |
| **Thời gian bỏ qua vạch ngang sau rẽ** | Cấu hình trong phần mềm (mặc định 200ms) | Bỏ qua các vạch ngang trong thời gian ngắn sau khi vừa vượt ngã rẽ để tránh quét đúp một vạch. |

---

## 4. Các Chế Độ Hoạt Động

ESP32 có thể yêu cầu STM32 chuyển đổi linh hoạt giữa 3 chế độ vận hành:

### 4.1 Chế Độ Lộ Trình Tự Hành (Autonomous Route Queue Mode)
Đây là chế độ làm việc chính thức khi xe phục vụ vận chuyển hàng:
1. ESP32 gửi gói tin nạp lộ trình `AGV_MSG_ROUTE_LOAD` (ID `0x10`):
   - Chứa số bước (`count`), vận tốc danh định (`target_rpm_x10`), và mảng hành động `actions[]`:
     - `0x01` (`AGV_ACTION_FORWARD`): Tiếp tục bám line chạy xuyên qua ngã rẽ.
     - `0x02` (`AGV_ACTION_TURN_LEFT`): Quay tại chỗ sang trái 90 độ cho đến khi bắt lại vạch nhánh.
     - `0x03` (`AGV_ACTION_TURN_RIGHT`): Quay tại chỗ sang phải 90 độ cho đến khi bắt lại vạch nhánh.
     - `0x00` (`AGV_ACTION_STOP`): Phanh dừng hoàn thành lộ trình tại trạm đích.
2. ESP32 gửi lệnh khởi động `AGV_MSG_ROUTE_START` (ID `0x11`).
3. STM32 bắt đầu di chuyển và phát `AGV_MSG_MISSION_STATUS` (`state = RUNNING`).
4. Khi đến trạm đích, STM32 phanh dừng và phát `AGV_MSG_MISSION_STATUS` (`state = COMPLETED`).
5. Nếu trong lúc chạy xe bị lệch khỏi vạch dẫn quá 1 giây, STM32 tự phanh dừng và phát `AGV_MSG_MISSION_STATUS` (`state = ERROR_LOST`).

### 4.2 Chế Độ Kiểm Thử & Chẩn Đoán Bàn Test (Diagnostics & Bench-Test Mode)
Dành cho người vận hành mở màn hình "Cài Đặt / Chẩn Đoán" trên ứng dụng điện thoại để kiểm tra xe ngay trên sa bàn:
- **Kiểm tra động cơ (Step Test)**: `AGV_MSG_DIAG_STEP_TEST` (`0x20`) — Chạy thử motor trái hoặc phải với mức PWM và thời gian ấn định để đo đạc đáp ứng gia tốc.
- **Đọc xung Encoder**: `AGV_MSG_DIAG_GET_ENC` (`0x23`) — STM32 trả về frame `AGV_MSG_DIAG_ENC_DATA` (`0xA3`) gồm xung 2 bánh `enc_left`, `enc_right`.
- **Reset xung Encoder**: `AGV_MSG_DIAG_RESET_ENC` (`0x24`) — Đặt lại xung encoder về 0.
- **Hiệu chuẩn cảm biến quang trở (Line Calib)**: `AGV_MSG_DIAG_LINE_TEST` (`0x26`, `mode = 1`) — Đặt xe lên nền trắng/đen để xe tự động học ngưỡng phân biệt màu.
- **Đặt tốc độ trực tiếp**: `AGV_MSG_DIAG_SET_SPEED` (`0x21`) hoặc `AGV_MSG_DIAG_SET_RPM` (`0x22`).
- **Tinh chỉnh thông số PI**: `AGV_MSG_DIAG_PI_TUNE` (`0x25`) — Cập nhật hệ số $K_p, K_i, K_{ff}$ điều khiển vận tốc từ xa mà không cần nạp lại firmware.

### 4.3 Chế Độ An Toàn Khẩn Cấp (Emergency Stop)
- Bất cứ khi nào người dùng ấn nút "DỪNG KHẨN CẤP" trên điện thoại hoặc xe phát hiện chướng ngại vật phía trước, ESP32 phải lập tức phát gói tin `AGV_MSG_EMERGENCY_STOP` (`0x02`).
- STM32 sẽ lập tức:
  - Hủy bỏ toàn bộ chuỗi lộ trình đang chạy.
  - Ngắt toàn bộ công suất điều khiển, phanh cứng 2 bánh xe (`BSP_Motor_Brake`).
  - Trả về gói phản hồi `AGV_MSG_ACK` (`0x80`).

---

## 5. Hướng Dẫn Thiết Kế Kiến Trúc Firmware Phía ESP32

Để đạt độ ổn định cao nhất khi chạy đa nhiệm trên ESP32 (kết nối BLE song song với truyền nhận UART thời gian thực), khuyến nghị cấu trúc theo mô hình **FreeRTOS Đa Luồng (Multi-Task Architecture)**:

```
┌────────────────────────────────────────────────────────────────────────┐
│ FREE-RTOS TRÊN ESP32-S3 (Xtensa LX7 @ 240MHz)                         │
├──────────────────────────────────┬─────────────────────────────────────┤
│ Core 0: Task Mạng & Giao Tiếp BLE │ Core 1: Task Điều Phối & UART       │
│                                  │                                     │
│  [Task 1: BLE Server]            │  [Task 2: Navigation FSM]           │
│  - Nhận lệnh di chuyển từ App    │  - Lưu sơ đồ các trạm (Graph Map)   │
│  - Phát Notify cập nhật tiến độ  │  - Tìm đường ngắn nhất (Dijkstra)   │
│  - Đẩy log debug lên điện thoại  │  - Đóng gói lệnh nhị phân UART      │
│                                  │                                     │
│                                  │  [Task 3: UART RX Handler]          │
│                                  │  - Đọc luồng byte từ UART2          │
│                                  │  - agv_parser_feed_byte()           │
│                                  │  - Xử lý gói tin ACK, EVENT, LOG    │
└──────────────────────────────────┴─────────────────────────────────────┘
```

### 5.1 Quản Lý Bản Đồ Sa Bàn & FSM Chuyển Bước
Trên ESP32, bạn có thể mô hình hóa sa bàn dưới dạng một mảng cấu trúc hoặc ma trận kề:

```cpp
// Ví dụ: Mô tả lộ trình giữa các trạm trên sa bàn của bạn
struct RouteMap {
    uint8_t from_station;
    uint8_t to_station;
    uint8_t num_steps;
    uint8_t actions[16];
};

const RouteMap g_station_routes[] = {
    // Đi từ Trạm 1 đến Trạm 2: Đi thẳng -> Rẽ phải -> Đi thẳng -> Dừng
    { 1, 2, 4, { AGV_ACTION_FORWARD, AGV_ACTION_TURN_RIGHT, AGV_ACTION_FORWARD, AGV_ACTION_STOP } },
    
    // Đi từ Trạm 2 đến Trạm 3: Đi thẳng -> Rẽ trái -> Dừng
    { 2, 3, 3, { AGV_ACTION_FORWARD, AGV_ACTION_TURN_LEFT, AGV_ACTION_STOP } },
    
    // Đi từ Trạm 3 về Trạm 1: Quay đầu / Rẽ trái -> Đi thẳng -> Dừng
    { 3, 1, 3, { AGV_ACTION_TURN_LEFT, AGV_ACTION_FORWARD, AGV_ACTION_STOP } }
};
```

Khi nhận lệnh BLE chọn trạm đích, ESP32 chỉ cần tra cứu bảng này, lấy mảng `actions` tương ứng rồi gọi `agv_pack_route_load(...)` gửi xuống STM32!

### 5.2 Xử Lý Kênh Log Debug Không Dây (`AGV_MSG_LOG`)
Trước đây xe dùng cổng ST-Link SEGGER RTT để in log. Hiện tại, thư viện RTT đã được gỡ bỏ hoàn toàn. Bất cứ khi nào STM32 chạy (nhịp tim Heartbeat, cảnh báo ngã rẽ, kết quả step test), nó đều gửi gói tin `AGV_MSG_LOG` (ID `0x99`) sang ESP32.
- **ESP32 chỉ cần**:
  ```c
  if (frame->msg_id == AGV_MSG_LOG) {
      char log_buf[65];
      uint8_t len = (frame->length < 64) ? frame->length : 64;
      memcpy(log_buf, frame->payload, len);
      log_buf[len] = '\0';
      
      // 1. In ra console debug của ESP32 qua cổng Type-C
      ESP_LOGI(TAG, "[STM32 LOG] %s", log_buf);
      
      // 2. Chuyển tiếp chuỗi log qua BLE Notify về máy tính/điện thoại
      ble_nus_send_string(log_buf);
  }
  ```
Nhờ đó, người vận hành hoàn toàn có thể theo dõi hành vi của xe khi đang chạy thực tế trên sàn mà không cần cắm bất kỳ sợi dây cáp nào vào xe.

---

## 6. Danh Mục Tài Liệu Tham Khảo

Toàn bộ tài nguyên phục vụ lập trình ESP32 đã có sẵn trong kho mã nguồn:

1. **Thư viện C giao thức dùng chung**:
   - [`Services/Inc/comms_protocol.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Inc/comms_protocol.h) — Khai báo các enum, struct packed và prototype hàm.
   - [`Services/Src/comms_protocol.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Src/comms_protocol.c) — Triển khai thuật toán CRC16-CCITT, máy trạng thái stream byte parser và các hàm đóng gói frame.
2. **Cẩm nang kỹ thuật giao thức UART**:
   - [`docs/esp32_uart_protocol_guide.md`](esp32_uart_protocol_guide.md) — Hướng dẫn chi tiết từng byte của các gói tin nhị phân.
3. **Biên bản ghi nhận quyết định kiến trúc**:
   - [`docs/decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md`](decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md) — Ghi lại toàn bộ bối cảnh, lý do lựa chọn và hệ quả kiến trúc của mô hình 2 MCU.
