# Tổng Quan Dự Án Xe Tự Hành AGV Dành Cho Lập Trình Viên ESP32

Tài liệu này được biên soạn dành riêng cho kỹ sư phát triển firmware trên **ESP32**, cung cấp cái nhìn toàn diện từ bức tranh tổng thể, kiến trúc phân tầng, nguyên lý chuyển động của xe AGV, cho đến hướng dẫn thiết kế máy trạng thái lộ trình (FSM) và giao tiếp với **STM32F103C8T6**.

---

## 1. Bức Tranh Tổng Thể Dự Án (Project Vision)

### 1.1 Chiếc Xe Tự Hành AGV Này Là Gì?
Đây là một robot tự hành công nghiệp cỡ nhỏ dạng **dẫn động vi sai hai bánh (Two-Wheel Differential Drive)** với mảng cảm biến quang học phía trước chuyên dùng để **chạy bám vạch kẻ sàn (Line Tracking)** trong nhà kho hoặc sa bàn mô phỏng:
- **Di chuyển**: Hai bánh chủ động độc lập điều khiển bằng động cơ DC giảm tốc kèm encoder quang học 2 kênh A/B, cùng 1 bánh xe tự hướng (caster wheel) trợ lực thăng bằng.
- **Dò đường**: Mảng 5 mắt quang trở hồng ngoại (Analog IR Array) gắn sát mặt sàn phía trước đầu xe để liên tục đo đạc vị trí lệch so với vạch dẫn hướng.
- **Sa bàn**: Vạch dẫn đường màu tương phản trên sàn. Tại các điểm giao nhau (ngã ba, ngã tư) hoặc điểm dừng trạm (Station A, B, C...) có vẽ **vạch ngang (Crossline / Intersection)** cắt ngang đường đi.

### 1.2 Mục Tiêu Vận Hành Hệ Thống
1. Người vận hành sử dụng ứng dụng di động (hoặc phần mềm quản lý kho) kết nối không dây qua **Bluetooth Low Energy (BLE)** với ESP32.
2. Người dùng phát lệnh: *"Xe hãy đi từ Trạm 1 đến Trạm 4"*.
3. **ESP32** (Bộ não điều phối) tra cứu bản đồ sa bàn, chuyển yêu cầu thành một chuỗi bước di chuyển: *Đi thẳng qua ngã 1 -> Rẽ phải tại ngã 2 -> Đi thẳng qua ngã 3 -> Dừng tại trạm 4*.
4. ESP32 nạp chuỗi lệnh này xuống **STM32** qua cổng UART nhị phân.
5. **STM32** (Tầng chấp hành) tự động bám line, tự đếm vạch ngang để chuyển hướng mượt mà, và dừng hẳn khi đến đích, sau đó bắn tín hiệu báo cho ESP32.
6. ESP32 cập nhật trạng thái lên giao diện điện thoại của người dùng.

---

## 2. Kiến Trúc 2 Vi Điều Khiển & Phân Chia Trách Nhiệm (Separation of Concerns)

Hệ thống tuân thủ nghiêm ngặt nguyên lý phân tách trách nhiệm giữa **Tầng Điều Phối Cấp Cao (High-Level Coordinator)** và **Tầng Chấp Hành Thời Gian Thực Cứng (Hard Real-Time Controller)**:

```
┌─────────────────────────────────────────────────────────────┐
│ ỨNG DỤNG ĐIỀU KHIỂN TRÊN ĐIỆN THOẠI (Mobile App / Web UI)   │
└──────────────────────────────┬──────────────────────────────┘
                               │ Bluetooth Low Energy (BLE)
┌──────────────────────────────▼──────────────────────────────┐
│ [ESP32] - BỘ NÃO ĐIỀU PHỐI CẤP CAO (High-Level Master)      │
│ 1. Gateway kết nối không dây BLE với người dùng.            │
│ 2. Quản lý bản đồ Topology (Đồ thị các trạm và nhánh rẽ).   │
│ 3. Lập kế hoạch đường đi (Route Planner / FSM).             │
│ 4. Chuyển tiếp log/chẩn đoán từ STM32 lên app người dùng.   │
│ 5. Phát lệnh Dừng khẩn cấp (Emergency Stop) khi có sự cố.   │
└──────────────────────────────┬──────────────────────────────┘
                               │ UART1 DMA 115200 8N1 (PA9/PA10)
                               │ Khung truyền nhị phân có CRC16
┌──────────────────────────────▼──────────────────────────────┐
│ [STM32] - BỘ ĐIỀU KHIỂN CHẤP HÀNH (Hard Real-Time Slave)    │
│ 1. Đọc ADC DMA 5 mắt cảm biến quang trở (Chu kỳ 10ms).      │
│ 2. Thuật toán bám vạch Line PID (Chu kỳ 10ms).              │
│ 3. Đo đạc xung Encoder & vòng lặp điều khiển vận tốc        │
│    động cơ kín Motor PI (Chu kỳ 10ms).                      │
│ 4. Autonomous Route Queue: Lưu chuỗi bước, tự đếm vạch      │
│    ngang IR và tự chuyển bước (không cần hỏi lại ESP32).    │
│ 5. Tự ngắt motor an toàn khi mất vạch liên tục (>1.0 giây). │
│ 6. Thực thi các bài kiểm tra phần cứng (Step Test, Diag).   │
└─────────────────────────────────────────────────────────────┘
```

### Tại Sao Lại Thiết Kế "Autonomous Route Queue"?
Trong quá trình thảo luận kiến trúc, câu hỏi đặt ra là: *Khi xe đến ngã rẽ, STM32 có nên hỏi ESP32 rẽ trái hay rẽ phải không?*
- **Câu trả lời là KHÔNG**: Nếu STM32 phải dừng lại, gửi câu hỏi qua UART, đợi ESP32 phân tích FSM rồi gửi lệnh rẽ xuống, xe sẽ bị **dừng giật cục tại mọi ngã rẽ**, trễ quán tính làm xe trôi vạch, và thời gian di chuyển giữa các trạm tăng gấp đôi.
- **Giải pháp tối ưu**: ESP32 gửi **toàn bộ danh sách bước từ đầu** (`AGV_MSG_ROUTE_LOAD`). Trong suốt hành trình, STM32 tự động quét vạch ngang và chuyển bước độc lập. Khi đến bước cuối cùng (hoặc gặp hành động `STOP`), xe tự phanh dừng và bắn gói tin `EVENT_MISSION_STATUS(COMPLETED)` lên ESP32!

---

## 3. Thông Số Vật Lý & Cơ Khí AGV (Physical Constants)

Lập trình viên ESP32 cần nắm các thông số cơ khí của xe để tính toán lộ trình và cấu hình vận tốc hợp lý:

| Thông Số Cơ Học / Điện | Giá Trị Thực Tế | Ý Nghĩa / Ảnh Hưởng Đến Lập Trình |
|---|---|---|
| **Đường kính bánh xe ($D$)** | $65.0\text{ mm}$ ($0.065\text{ m}$) | Chu vi bánh xe: $C = \pi \times D \approx 204.2\text{ mm}$ mỗi vòng. |
| **Khoảng cách 2 bánh ($L$)** | $160.0\text{ mm}$ ($0.160\text{ m}$) | Bán kính quay đầu vi sai: $R_{turn} = L / 2 = 80\text{ mm}$. |
| **Độ phân giải Encoder ($CPR$)** | $1320\text{ xung/vòng}$ | Đọc qua Timer 16-bit chế độ Encoder Mode 4X. |
| **Vận tốc danh định tiêu chuẩn** | **`80.0 RPM`** | Tương đương vận tốc dài $\approx 0.27\text{ m/s}$ (rất ổn định trên sa bàn). |
| **Dải vận tốc hỗ trợ** | `40.0` đến `120.0 RPM` | Khi rẽ ngã ba/ngã tư, STM32 tự giảm tốc xuống 60% ($\approx 48\text{ RPM}$) để đảm bảo an toàn. |
| **Mức điều khiển PWM TB6612** | `-1000` đến `+1000` | Điều chế độ rộng xung 20kHz, tương ứng $-100.0\%$ đến $+100.0\%$ công suất. |
| **Thời gian tự ngắt mất vạch** | `1000 ms` ($1.0\text{ giây}$) | Nếu cả 5 mắt IR mất vạch quá 1s, STM32 tự dừng khẩn cấp và báo lỗi `ERROR_LOST`. |
| **Thời gian debounce ngã rẽ** | `400 ms` | STM32 tự bỏ qua các vạch ngang trong 400ms sau ngã rẽ để không bị quét đúp 1 vạch. |

---

## 4. Các Chế Độ Hoạt Động (Operating Modes)

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
│ FREE-RTOS TRÊN ESP32 (Dual-Core Xtensa LX6 @ 240MHz)                   │
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
Trước đây xe dùng cổng ST-Link SEGGER RTT để in log. Hiện tại, thư viện RTT đã được **gỡ bỏ hoàn toàn**. Bất cứ khi nào STM32 chạy (nhịp tim Heartbeat, cảnh báo ngã rẽ, kết quả step test), nó đều gửi gói tin `AGV_MSG_LOG` (ID `0x99`) sang ESP32.
- **ESP32 chỉ cần**:
  ```cpp
  if (frame->msg_id == AGV_MSG_LOG) {
      char log_buf[65];
      uint8_t len = (frame->length < 64) ? frame->length : 64;
      memcpy(log_buf, frame->payload, len);
      log_buf[len] = '\0';
      
      // 1. In ra Serial Monitor của ESP32 qua cổng Type-C
      Serial.printf("[STM32 LOG] %s\r\n", log_buf);
      
      // 2. (Tùy chọn) Bắn chuỗi log này qua BLE Characteristic về điện thoại
      pLogCharacteristic->setValue(log_buf);
      pLogCharacteristic->notify();
  }
  ```
Nhờ đó, bạn hoàn toàn có thể theo dõi "sức khỏe" và hành vi của xe khi đang chạy thực tế trên sàn mà không cần cắm bất kỳ sợi dây cáp nào vào xe!

---

## 6. Danh Mục Tài Liệu & File Mã Nguồn Tham Khảo

Toàn bộ tài nguyên phục vụ lập trình ESP32 đã có sẵn trong kho mã nguồn:

1. **Thư viện C giao thức dùng chung**:
   - [`Services/Inc/comms_protocol.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Inc/comms_protocol.h) — Khai báo các enum, struct packed và prototype hàm.
   - [`Services/Src/comms_protocol.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Services/Src/comms_protocol.c) — Triển khai thuật toán CRC16-CCITT, máy trạng thái stream byte parser và các hàm đóng gói frame.
2. **Cẩm nang kỹ thuật & Code mẫu ESP32**:
   - [`docs/esp32_uart_protocol_guide.md`](esp32_uart_protocol_guide.md) — Hướng dẫn chi tiết từng byte của các gói tin và file code mẫu C++ hoàn chỉnh cho ESP32.
3. **Biên bản ghi nhận quyết định kiến trúc**:
   - [`docs/decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md`](decisions/ADR-003-esp32-gateway-binary-protocol-and-rtt-removal.md) — Ghi lại toàn bộ bối cảnh, lý do lựa chọn và hệ quả kiến trúc của mô hình 2 MCU.
