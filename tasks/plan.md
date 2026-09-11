# Kế Hoạch Triển Khai: Chuyển Đổi Tầng BLE Gateway Sang Freenove ESP32-S3 CAM

## 1. Tổng Quan (Overview)
Mục tiêu là di chuyển toàn bộ phần mềm Gateway không dây `agv_esp32` từ vi điều khiển ESP32 classic sang **Freenove ESP32-S3 CAM** (chip ESP32-S3R8 / Xtensa LX7 dual-core). Quá trình này bảo toàn 100% kiến trúc 4 tầng, giao thức nhị phân khung truyền (`comms_protocol`), các dịch vụ BLE NUS, nút bấm BOOT và bộ công cụ chẩn đoán không dây trên máy tính (`agv_ble_controller.py`).

Cặp chân giao tiếp UART với STM32 (PA9/PA10) được chuẩn hóa sang **GPIO 42 (TX)** và **GPIO 41 (RX)** để cách ly hoàn toàn, không xung đột với các bus ngoại vi tích hợp sẵn trên kit Freenove (Camera OV2640, Octal PSRAM, thẻ nhớ SD, LED báo trạng thái). Cổng kết nối nạp và console log được cố định qua cổng Type-C **UART** (chip USB-Serial trên board).

---

## 2. Quyết Định Kiến Trúc & Phần Cứng (Architecture Decisions)

1. **Bộ điều khiển vi xử lý mục tiêu (Target SoC)**:
   - Chuyển `target` từ `esp32` sang `esp32s3` trên môi trường **ESP-IDF v6.0.2**.
   - Ngăn xếp Bluetooth: NimBLE BLE 5.0 (`CONFIG_BT_NIMBLE_ENABLED=y`), ATT MTU $\ge 256$ bytes, không bật BR/EDR (S3 chỉ hỗ trợ phần cứng BLE).

2. **Quy hoạch chân GPIO trên Freenove ESP32-S3 CAM**:
   - **UART kết nối STM32**:
     - `ESP32-S3 GPIO 42 (TX)` ➔ `STM32 PA10 (RX)`
     - `ESP32-S3 GPIO 41 (RX)` ➔ `STM32 PA9 (TX)`
     - `GND` chung.
     - Sử dụng ngoại vi `UART_NUM_1` (hoặc `UART_NUM_2`), baudrate 115200 8N1, cơ chế nhận luồng zero-malloc FreeRTOS task.
   - **Cổng nạp & Console Log**:
     - Cổng Type-C nhãn **UART** (nối qua chip USB-to-UART vào UART0 GPIO 43/44) dùng cho `idf.py flash` và `idf.py monitor`.
   - **Nút nhấn BOOT**:
     - Giữ nguyên `GPIO_NUM_0` (trên kit Freenove S3 CAM, nút BOOT vật lý gắn trực tiếp vào GPIO 0).
   - **Bảo toàn chân cho tương lai (Camera / SD / RAM)**:
     - Dành trọn vẹn dải GPIO 4–18 cho bus Camera DVP (OV2640).
     - Tránh GPIO 35–37 (dành riêng cho 8MB Octal PSRAM OPI).
     - Tránh GPIO 38–40 (dành riêng cho khe cắm thẻ MicroSD 1-bit SDMMC).
     - Tránh GPIO 2 (LED trên board) và GPIO 48 (WS2812 RGB LED).

3. **Tính tương thích phần mềm**:
   - **STM32 Firmware (`agv_stm32`)**: Không thay đổi bất kỳ dòng mã nào. Giao thức nhị phân `comms_protocol.c/.h` hoàn toàn tương thích nhị phân (Little-Endian, CRC16-CCITT).
   - **PC Python Tool (`agv_esp32/tools/agv_ble_controller.py`)**: Giữ nguyên toàn bộ 15 lệnh CLI và các tính năng chẩn đoán (`monitor_ir`, `calibrate_line`, `step`).

---

## 3. Danh Sách Giai Đoạn & Nhiệm Vụ (Task List)

### Giai Đoạn 1: Tái Cấu Hình Target & Thích Ứng Mã Nguồn Phần Cứng
- [ ] **Task 1**: Cập nhật cấu hình ESP-IDF Target sang `esp32s3` và tinh chỉnh `sdkconfig.defaults`.
- [ ] **Task 2**: Cập nhật pinout UART sang GPIO 42 (TX) & GPIO 41 (RX) trong `main.c` và `uart_service`.

### Checkpoint 1: Biên Dịch Thành Công Trên ESP-IDF v6.0.2
- [ ] Xóa cache build cũ và biên dịch sạch sẽ (`idf.py build`) cho target `esp32s3`.
- [ ] Không có lỗi biên dịch (0 errors, 0 critical warnings).

### Giai Đoạn 2: Đồng Bộ Tài Liệu & Kiểm Thử Tương Thích
- [ ] **Task 3**: Cập nhật tài liệu kỹ thuật (`SPEC-agv-esp32-binary-gateway.md` và `README.md`) với sơ đồ đấu nối Freenove S3 CAM.
- [ ] **Task 4**: Chạy bộ kiểm thử tự động (Python BLE Controller tests & STM32 protocol unit tests).

### Checkpoint 2: Hoàn Tất & Sẵn Sàng Vận Hành
- [ ] 100% unit tests vượt qua (22/22 BLE controller tests, 110/110 STM32 tests).
- [ ] File nhị phân sẵn sàng nạp: `agv_esp32/build/agv_esp32.bin`.
- [ ] Sơ đồ nối dây chi tiết sẵn sàng cho người dùng thao tác phần cứng.

---

## 4. Quản Trị Rủi Ro (Risks and Mitigations)

| Rủi ro | Mức độ | Biện pháp giảm thiểu |
| :--- | :---: | :--- |
| **Xung đột cấu hình SDK cũ** | Trung bình | Xóa thư mục `agv_esp32/build` và file `sdkconfig` cũ trước khi chạy `idf.py set-target esp32s3` để đảm bảo Kconfig sinh mới hoàn toàn. |
| **Trùng chân Strapping pin trên S3** | Thấp | Cặp chân GPIO 41 và GPIO 42 đã được kiểm chứng không thuộc dải strapping pins (strapping pins của S3 là GPIO 0, 3, 45, 46). |
| **Mất cổng COM ảo khi chip reset** | Thấp | Người dùng đã thống nhất cắm cổng Type-C nhãn **UART** (qua chip CH340), giúp cổng COM luôn ổn định và không bị ngắt quãng như native USB CDC. |
