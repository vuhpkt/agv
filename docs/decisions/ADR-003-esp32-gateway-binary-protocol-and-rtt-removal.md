# ADR-003: Kiến Trúc 2 MCU (ESP32 Gateway + STM32 Motion Controller), Giao Thức Nhị Phân UART và Loại Bỏ SEGGER RTT

## Trạng thái (Status)
**Đã chấp thuận (Accepted)**

## Ngày quyết định (Date)
2026-09-09

## Bối cảnh (Context)
Trong giai đoạn đầu phát triển, xe AGV STM32F103 được kiểm thử trên bàn test (Bench Testing) thông qua mạch nạp ST-Link và cổng SEGGER RTT tốc độ cao để nhận lệnh chuỗi ký tự ASCII (`PING`, `RPM=80,80`, `STEP_TEST`, `LINE_TEST`) và in telemetry.

Khi chuyển sang giai đoạn vận hành thực địa trên sa bàn (Untethered Field Operation):
1. **Mất kết nối vật lý có dây**: Xe tự hành chạy bằng pin dưới sàn, cáp ST-Link bị rút ra khiến SEGGER RTT trở nên vô dụng.
2. **Nhu cầu điều khiển không dây**: Xe cần kết nối không dây (Bluetooth Low Energy - BLE) tới điện thoại hoặc máy tính của người vận hành để nhận lệnh di chuyển giữa các trạm (Station A, B, C...).
3. **Cần phân định ranh giới trách nhiệm giữa 2 MCU**: ESP32 sở hữu Wi-Fi/BLE và tài nguyên bộ nhớ lớn, trong khi STM32F103 có thế mạnh về điều khiển ngoại vi thời gian thực cứng (Hard Real-Time Timer, ADC DMA, Motor PWM, Encoder). Cần một kiến trúc phân tầng rõ ràng để tránh chồng chéo logic.
4. **Độ trễ và an toàn tại các ngã rẽ**: Nếu mỗi lần gặp vạch ngang ngã rẽ, STM32 phải dừng lại, gửi câu hỏi lên ESP32, đợi ESP32 phản hồi rồi mới rẽ thì xe sẽ bị giật cục, quán tính trôi vạch hoặc trễ truyền thông làm lệch lộ trình.
5. **Nhiễu điện từ trên đường truyền UART**: Động cơ DC công suất lớn khi đảo chiều hoặc thay đổi tải có thể phát xung nhiễu điện từ vào đường truyền UART TTL nối giữa 2 bo mạch.

---

## Quyết định (Decision)

Dự án quyết định thiết lập mô hình **Phân tầng 2 MCU (2-Tier Architecture)**, sử dụng **Giao thức Khung truyền Nhị phân (Binary Frame)** trên UART1 DMA và **Gỡ bỏ hoàn toàn thư viện SEGGER RTT**:

```
┌────────────────────────────────────────────────────────┐
│ NGƯỜI DÙNG / APP ĐIỀU KHIỂN TRÊN ĐIỆN THOẠI            │
└───────────────────────────┬────────────────────────────┘
                            │ (Bluetooth Low Energy - BLE)
┌───────────────────────────▼────────────────────────────┐
│ TẦNG ĐIỀU PHỐI CẤP CAO (ESP32 - Master Coordinator)     │
│ - Quản lý kết nối BLE & Giao diện người dùng           │
│ - Quản lý bản đồ Topology sa bàn (Graph of Stations)   │
│ - Máy trạng thái lộ trình (FSM Route Planner):         │
│   Phân tích Trạm A -> Trạm B thành chuỗi:               │
│   [FORWARD, TURN_RIGHT, FORWARD, STOP]                 │
└───────────────────────────┬────────────────────────────┘
                            │ UART1 TTL (115200 8N1 DMA Circular)
                            │ Khung truyền nhị phân (CRC16-CCITT)
┌───────────────────────────▼────────────────────────────┐
│ TẦNG CHẤP HÀNH THỜI GIAN THỰC (STM32 - Slave Controller)│
│ - Autonomous Route Queue: Nhận chuỗi bước di chuyển,   │
│   tự đếm vạch ngang IR và tự chuyển bước mượt mà       │
│ - Điều khiển vòng kín động cơ (Motor PI Loop 100Hz)     │
│ - Xử lý tín hiệu mảng quang trở IR & Line PID (100Hz)  │
│ - Tự ngắt an toàn khi mất vạch liên tục (>1.0 giây)     │
│ - Báo cáo EVENT_MISSION_STATUS & Đóng gói MSG_LOG       │
└────────────────────────────────────────────────────────┘
```

### 1. Phân Chia Ranh Giới Trách Nhiệm (Separation of Concerns)
- **ESP32 (High-Level Brain)**:
  - Giữ bản đồ vị trí trạm và lộ trình tổng thể.
  - Quyết định xe cần đi qua những đoạn nào để đến đích.
  - Nạp toàn bộ chuỗi hành động một lần từ đầu (`AGV_MSG_ROUTE_LOAD`), sau đó ra lệnh chạy (`AGV_MSG_ROUTE_START`).
  - Lắng nghe sự kiện hoàn thành (`MISSION_COMPLETED`) hoặc sự cố (`MISSION_ERROR_LOST`) từ STM32 để thông báo cho người dùng.
  - Cho phép người dùng gửi lệnh Dừng khẩn cấp (`AGV_MSG_EMERGENCY_STOP`) hoặc thực thi các bài test chẩn đoán bàn test (`AGV_MSG_DIAG_*`).
- **STM32 (Hard Real-Time Motion Controller)**:
  - Lưu trữ mảng hành vi tĩnh (`route_actions[16]`).
  - Khi đang chạy theo vạch và gặp vạch ngang: **tự động thi hành bước tiếp theo trong hàng đợi mà không cần gửi request hỏi lại ESP32**.
  - Không cần lưu bản đồ tọa độ phức tạp, giữ cho firmware STM32 thuần túy là bộ điều khiển thời gian thực chính xác từng mili-giây.

### 2. Giao Thức Nhị Phân Chuẩn Hóa C Thuần (Zero-HAL)
- Khung truyền: `[0xAA 0x55] [MSG_ID] [LENGTH] [PAYLOAD] [CRC16_H] [CRC16_L]`.
- Bảo vệ toàn vẹn bằng **CRC16-CCITT** (Poly `0x1021`, Init `0xFFFF`), loại bỏ hoàn toàn nguy cơ nhận nhầm byte rác do nhiễu động cơ.
- Cung cấp module `comms_protocol.h` / `comms_protocol.c` thuần C, độc lập nền tảng, cho phép copy trực tiếp sang project ESP32 dùng chung mà không phải viết lại code parser.

### 3. Gỡ Bỏ SEGGER RTT & Chuyển Đổi Kênh Log Sang Nhị Phân
- Xóa bỏ mã nguồn và include `SEGGER_RTT` khỏi `CMakeLists.txt` để tiết kiệm Flash và RAM.
- Toàn bộ log debug trong firmware được bọc vào macro `AGV_LOG(...)`, tự động đóng gói thành gói tin nhị phân `AGV_MSG_LOG` (`0x99`) gửi sang ESP32 qua UART1 DMA. Kỹ sư có thể xem log từ xa qua cổng Serial của ESP32 hoặc qua ứng dụng BLE trên điện thoại.

---

## Các giải pháp thay thế đã cân nhắc (Alternatives Considered)

### Giải pháp 1: Giữ giao thức chuỗi văn bản ASCII CLI (`PING`, `RPM=80,80`)
- **Ưu điểm**: Dễ test bằng cách cắm mạch USB-TTL vào máy tính và gõ Terminal.
- **Nhược điểm**: Tốn CPU và RAM cho các hàm xử lý chuỗi (`sscanf`, `strtok`, `snprintf`), dễ tràn bộ đệm, không có cơ chế Checksum/CRC kiểm tra lỗi đường truyền, khó biểu diễn mảng danh sách lộ trình nhiều bước.
- **Lý do bác bỏ**: Không phù hợp cho giao tiếp trực tiếp MCU-to-MCU trong môi trường công nghiệp có nhiễu động cơ.

### Giải pháp 2: Cơ chế Hỏi - Đáp tại từng ngã rẽ (Request-Response per Intersection)
- **Cơ chế**: Khi STM32 thấy vạch ngang thì phanh dừng lại, gửi gói `INTERSECTION_FOUND` lên ESP32; ESP32 kiểm tra FSM rồi gửi lệnh `TURN_RIGHT` xuống; STM32 nhận lệnh mới bắt đầu rẽ.
- **Ưu điểm**: ESP32 kiểm soát chặt chẽ từng mét di chuyển của xe.
- **Nhược điểm**: Xe bị dừng giật cục tại mọi ngã rẽ, mất trớn quán tính, thời gian hoàn thành lộ trình tăng gấp đôi. Nếu ESP32 bận tác vụ BLE làm trễ phản hồi UART vài chục mili-giây, xe có thể vọt qua vạch rẽ.
- **Lý do bác bỏ**: Không đáp ứng yêu cầu chuyển động mượt mà và tin cậy thời gian thực của xe tự hành.

### Giải pháp 3: Nhúng toàn bộ bản đồ và FSM vào STM32, ESP32 chỉ làm cầu nối pass-through
- **Ưu điểm**: Giảm thiểu giao tiếp giữa 2 MCU.
- **Nhược điểm**: Bộ nhớ STM32F103 (64KB Flash, 20KB RAM) bị phình to do phải chứa cấu trúc dữ liệu bản đồ, thuật toán tìm đường (Dijkstra/A*), và khó thay đổi bản đồ khi sa bàn mở rộng.
- **Lý do bác bỏ**: Vi phạm nguyên lý phân tách trách nhiệm; ESP32 có dung lượng bộ nhớ lớn hơn gấp hàng chục lần (4MB Flash, 520KB RAM) là nơi lý tưởng để quản lý bản đồ và FSM cấp cao.

---

## Hệ quả (Consequences)

### Tích cực (Positive)
- **Độ trễ ngã rẽ bằng 0**: STM32 chuyển bước lộ trình ngay lập tức khi mắt quang trở chạm vạch ngang.
- **Độ tin cậy tối đa**: Mã CRC16-CCITT giúp hệ thống tự động loại bỏ các gói tin lỗi do nhiễu motor, không bao giờ nhận nhầm hướng rẽ.
- **Tối ưu tài nguyên STM32**: RAM STM32 chỉ chiếm 26.6% (5.4KB), Flash chỉ chiếm 80.08%, không còn tốn RAM cho bộ đệm RTT hay parser chuỗi văn bản.
- **Tiện lợi cho lập trình viên ESP32**: Dùng chung file header/source C `comms_protocol.h` / `comms_protocol.c`, không cần tự parse byte bằng tay.
- **Debug không dây thuận tiện**: Toàn bộ log của STM32 được ESP32 đón nhận và hiển thị trực tiếp lên điện thoại hoặc máy tính.

### Hạn chế & Thách thức (Trade-offs & Mitigations)
- **Cần cơ chế Debounce ngã rẽ**: Khi xe đi qua vạch ngang có độ dày lớn, nếu không có debounce thời gian (chặn 400ms), cảm biến có thể quét vạch ngang 2 lần gây nhảy cóc bước. *Đã giải quyết bằng cờ thời gian `s_crossline_debounce_until` trong `app_line_follow.c`.*
- **Phụ thuộc vào trạm đích**: Nếu vạch ngang cuối cùng bị mờ hoặc xe lệch khỏi vạch, xe có thể không nhận diện được trạm dừng. *Đã giải quyết bằng cơ chế timeout tự ngắt an toàn `LINE_LOST_TIMEOUT_MS = 1000`.*
- **Nối dây phần cứng**: Bắt buộc phải nối chung chân GND giữa STM32 và ESP32 để tránh sai lệch mức logic 3.3V UART.
