# Spec: AGV Python BLE Desktop GUI Dashboard

## Phase 0: Capability Map

| Module ID | Responsibility | Depends On |
|---|---|---|
| `gui-ble-worker` | Luồng nền `AsyncBleWorker` chạy `asyncio` & `bleak` trong một `threading.Thread`: quét tìm `AGV_ESP32`, kết nối/ngắt kết nối BLE NUS, gửi lệnh và nhận chuỗi text đẩy vào hàng đợi thread-safe `queue.Queue`. | `bleak` |
| `gui-telemetry-engine` | Bộ phân tích cú pháp dữ liệu thời gian thực: phân tách telemetry 5 mắt IR (`DATA:LINE,...`), chuỗi điểm đáp ứng bước nhảy (`DATA:<ms>,...`), phản hồi lệnh (`[RESP]`, `[ACK]`, `[ERR]`), trạng thái encoder và log hệ thống. | — |
| `gui-view-dashboard` | Giao diện đồ họa người dùng bằng `customtkinter`: Header kết nối & nút Dừng khẩn cấp, Tab 1 Giám sát mảng 5 mắt IR & Điều khiển tay, Tab 2 Đồ thị Step Response nhúng Matplotlib, Tab 3 Hiệu chuẩn quang học, Tab 4 Lập trình lộ trình, Console log thời gian thực. | `gui-ble-worker`, `gui-telemetry-engine`, `customtkinter`, `matplotlib` |
| `gui-test-suite` | Bộ kiểm thử tự động `test_ble_gui.py`: kiểm tra parser telemetry, logic chuyển đổi lệnh, thread-safe queue và bộ lọc dữ liệu. | `unittest` |

---

## 1. Objective (Mục Tiêu)

Xây dựng ứng dụng giao diện đồ họa Desktop hiện đại (`agv_esp32/tools/agv_ble_gui.py`) bằng thư viện **CustomTkinter** và **Matplotlib** kết nối không dây Bluetooth Low Energy (BLE) với **Freenove ESP32-S3 CAM**. Giao diện này thay thế hoàn toàn việc gõ lệnh dòng lệnh CLI truyền thống, mang lại trải nghiệm trực quan cho việc vận hành, hiệu chuẩn và chẩn đoán phần cứng robot AGV trên bàn test.

### User Stories & Acceptance Criteria

1. **US-1: Quét & Kết Nối BLE 1-Click**
   - *Khi* người dùng mở app và bấm "Scan & Connect".
   - *Thì* worker quét tìm thiết bị có tên `AGV_ESP32`, tự động đàm phán MTU $\ge 256$, cập nhật badge trạng thái sang màu xanh lá `CONNECTED`, hiển thị RSSI và kích hoạt các nút điều khiển.
2. **US-2: Dừng Khẩn Cấp Tức Thì (Safety Button)**
   - *Khi* người dùng bấm nút đỏ to bản "EMERGENCY STOP" (hoặc phím tắt Spacebar).
   - *Thì* ứng dụng lập tức phát lệnh `STOP`, gửi frame `AGV_MSG_EMERGENCY_STOP` xuống STM32 phanh cứng 2 bánh và đổi giao diện sang cảnh báo an toàn.
3. **US-3: Giám Sát Thời Gian Thực Mảng 5 Mắt IR (Visual Bar Meter)**
   - *Khi* người dùng bấm "Start Monitor" (phát lệnh `MONITOR_IR`).
   - *Thì* ứng dụng đón nhận các dòng `DATA:LINE,...` ở tần số 20Hz, cập nhật trực tiếp 5 thanh Progress Bar đồ họa đại diện cho 5 cảm biến, hiển thị sai số tâm line $e_{line}$ và đèn báo "Line Detected".
4. **US-4: Chẩn Đoán & Vẽ Đồ Thị Đáp Ứng Bước Nhảy Động Cơ (Embedded Plot)**
   - *Khi* người dùng chọn động cơ (Trái/Phải), kéo thanh trượt PWM (-1000..1000), đặt thời gian và bấm "Run Step Test".
   - *Thì* ứng dụng phát lệnh `STEP_TEST`, tích lũy các dòng điểm dữ liệu thời gian thực và vẽ trực tiếp đồ thị vận tốc RPM theo thời gian ngay trên khung Matplotlib trong ứng dụng; tự tính toán $RPM_{peak}$, $RPM_{ss}$, và hỗ trợ xuất file CSV.
5. **US-5: Hiệu Chuẩn Quang Học 1-Click (Calibration Wizard)**
   - *Khi* người dùng bấm "Start Calibration" (lệnh `LINE_CALIB`).
   - *Thì* ứng dụng theo dõi dải quét Min/Max của từng cảm biến, kiểm tra điều kiện dải động $\Delta \ge 100$, in bảng tóm tắt và cho phép bấm "Save Header" để cập nhật trực tiếp vào `app_line_calib_defaults.h`.
6. **US-6: Lập Trình & Giám Sát Lộ Trình Tự Hành (Mission Planner)**
   - *Khi* người dùng tạo chuỗi bước ngã rẽ (`FORWARD`, `TURN_RIGHT`, `TURN_LEFT`, `STOP`) và bấm "Start Mission".
   - *Thì* ứng dụng nạp lộ trình (`ROUTE_LOAD`), kích hoạt chạy (`ROUTE_START`) và cập nhật tiến trình bước đang chạy qua gói `[MISSION] State: RUNNING, Step: X`.
7. **US-7: Console Log Lọc Đa Kênh**
   - Khung log ở đáy màn hình hiển thị toàn bộ log từ ESP32 và STM32, hỗ trợ lọc theo thẻ (`All`, `Logs`, `Telemetry`, `Commands`) và có ô nhập lệnh tự do kèm phím Enter.

---

## 2. Kiến Trúc Đa Luồng & Xử Lý Async (Architecture & Thread Safety)

Do `customtkinter` (dựa trên Tkinter) chạy vòng lặp sự kiện đồ họa `root.mainloop()` đồng bộ (blocking), trong khi thư viện `bleak` bắt buộc phải chạy trong môi trường bất đồng bộ `asyncio`, ta áp dụng kiến trúc **Producer-Consumer Thread-Safe**:

```
┌─────────────────────────────────────────────────────────────┐
│  GUI MAIN THREAD (CustomTkinter Event Loop - 60 FPS)        │
│  - Render widgets, nút bấm, thanh bar cảm biến, đồ thị      │
│  - Định kỳ mỗi 20ms: kiểm tra `gui_rx_queue` qua after()    │
│  - Khi người dùng tương tác: đẩy lệnh vào `gui_tx_queue`    │
└─────────────────────▲─────────────────────────┬─────────────┘
                      │                         │
         queue.Queue  │ Event Frames            │ Commands
         (Thread-Safe)│                         │ (Thread-Safe)
                      │                         ▼
┌─────────────────────┴───────────────────────────────────────┐
│  BACKGROUND THREAD (Asyncio BLE Worker Loop)                │
│  - Chạy `asyncio.run(worker.main_loop())` độc lập           │
│  - Đọc `gui_tx_queue` và gọi `client.write_gatt_char()`     │
│  - Lắng nghe notification BLE NUS từ ESP32-S3               │
│  - Phân tích cú pháp sơ bộ và đẩy vào `gui_rx_queue`       │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Tech Stack & Dependencies

- **Ngôn ngữ**: Python 3.10+ (sử dụng Python 3.14 có sẵn).
- **Thư viện GUI**: `customtkinter >= 6.0.0` (Dark theme, bo góc hiện đại).
- **Thư viện BLE**: `bleak >= 0.21.0` (Giao tiếp Bluetooth Low Energy).
- **Thư viện Đồ họa**: `matplotlib >= 3.8.0` (Nhúng canvas `FigureCanvasTkAgg` vào Tkinter).
- **Thư viện Kiểm thử**: `unittest` (Bộ test tự động cho Telemetry Parser & Command Formatter).

---

## 4. Success Criteria (Tiêu Chí Nghiệm Thu)

- [ ] **SC-1 (Giao diện Khởi động)**: Khởi chạy `py -3 agv_esp32/tools/agv_ble_gui.py` hiển thị ngay cửa sổ Dashboard Dark Mode sắc nét, không bị đơ giật UI.
- [ ] **SC-2 (Quét & Kết nối BLE)**: Quét và kết nối thành công với `AGV_ESP32`, hiển thị trạng thái kết nối và MTU.
- [ ] **SC-3 (Nút Dừng Khẩn Cấp)**: Nút Stop màu đỏ phát lệnh `STOP` tức thời trong mọi hoàn cảnh.
- [ ] **SC-4 (Thanh đo IR trực quan)**: Khi stream IR, 5 thanh bar thay đổi kích thước và màu sắc mượt mà theo giá trị ADC thực tế.
- [ ] **SC-5 (Đồ thị Step Test)**: Chạy bài test động cơ, đồ thị tự động vẽ đường cong đặc tính RPM theo mili-giây.
- [ ] **SC-6 (Unit Test 100%)**: Bộ test tự động `test_ble_gui.py` kiểm thử toàn bộ logic phân tích cú pháp và dispatch lệnh đạt 100% PASS.
