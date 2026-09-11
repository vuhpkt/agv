# ADR-004: Chuyển Đổi Gateway Sang Freenove ESP32-S3 CAM, Quy Hoạch Pinout UART1 Cách Ly Camera & PSRAM

## Trạng thái (Status)
**Đã chấp thuận (Accepted)**

## Ngày quyết định (Date)
2026-09-11

## Bối cảnh (Context)
Trong kiến trúc 2-Tier Dual-MCU (ADR-003), ESP32 đảm nhiệm vai trò Gateway không dây BLE (NimBLE Nordic UART Service) và điều phối cấp cao kết nối với bộ điều khiển chuyển động STM32F103 qua giao thức nhị phân `comms_protocol`. Ban đầu, firmware được xây dựng cho chip ESP32 classic (WROOM-32) với chân UART2 mặc định là `GPIO16 (RX)` và `GPIO17 (TX)`.

Khi dự án mở rộng:
1. **Mục tiêu tương lai về Camera và Thị Giác Máy Tính**: Xe AGV cần khả năng bổ sung module camera (như OV2640 / OV5640) để nhận diện vạch dẫn phức tạp, đọc mã QR trạm hoặc stream hình ảnh. Chip ESP32 classic có băng thông hạn chế và không có tập lệnh vector AI.
2. **Sẵn có phần cứng Freenove ESP32-S3 CAM**: Kit phát triển Freenove ESP32-S3 WROOM CAM (ESP32-S3R8, Xtensa LX7 dual-core @ 240MHz, 8MB Octal OPI PSRAM, khe cắm MicroSD, cổng cắm camera chuyên dụng) đã có sẵn trên bàn test.
3. **Xung đột phần cứng nghiêm trọng nếu giữ nguyên pinout**:
   - Trên kit Freenove ESP32-S3 CAM, cặp chân `GPIO16` và `GPIO17` đã bị đấu nối cứng bên trong bo mạch vào bus dữ liệu camera DVP (lần lượt là chân `CAM_Y9` và `CAM_Y8` / D7 và D6). Nếu tiếp tục dùng GPIO16/17 làm UART kết nối STM32, camera sẽ bị tê liệt hoặc gây đoản mạch bus dữ liệu khi cắm module camera sau này.
   - Các chân `GPIO35, 36, 37` bị chiếm dụng bởi 8MB Octal PSRAM.
   - Các chân `GPIO38, 39, 40` bị chiếm dụng bởi khe cắm thẻ nhớ MicroSD.
   - Chân `GPIO2` nối đèn LED nguồn onboard và `GPIO48` nối đèn WS2812 RGB LED.
   - Các chân `GPIO0, 3, 45, 46` là strapping pins của ESP32-S3.
4. **Cổng nạp và Console Log**: Bo mạch tích hợp 2 cổng USB Type-C: cổng "UART" (qua chip nạp USB-to-UART CH340 nối UART0 GPIO43/44) và cổng "USB" (native USB CDC GPIO19/20). Cần xác định cổng kết nối chuẩn để tránh mất kết nối log khi chip reset.

---

## Quyết định (Decision)

Dự án quyết định chuyển đổi toàn bộ tầng Gateway `agv_esp32` sang **Freenove ESP32-S3 CAM** (target `esp32s3` trên ESP-IDF v6.0.2), quy hoạch lại chân UART sang **GPIO 42 (TX) & GPIO 41 (RX)** và bảo toàn toàn bộ tài nguyên cho camera tương lai:

### 1. Quy Hoạch Chân Kết Nối Phần Cứng (Pinout Allocation)

| Tín Hiệu / Chức Năng | Chân ESP32-S3 | Chân STM32F103 | Ghi Chú Kỹ Thuật |
| :--- | :---: | :---: | :--- |
| **UART TX (ESP32 ➔ STM32)** | **GPIO 42** | **PA10 (RX)** | UART1 TX, Non-strapping pin, hàng header ngoài, an toàn tuyệt đối. |
| **UART RX (ESP32 🠄 STM32)** | **GPIO 41** | **PA9 (TX)** | UART1 RX, Pull-up nội, độc lập với bus Camera và thẻ SD. |
| **GND Chung** | **GND** | **GND** | Mass chuẩn chung giữa 2 vi điều khiển. |
| **Nút BOOT PING** | **GPIO 0** | — | Nút cứng BOOT onboard (Active-LOW, debounce 50ms), ấn để gửi PING. |
| *Bảo lưu: Camera OV2640* | GPIO 4–18 | — | Bus DVP camera (XCLK, PCLK, VSYNC, HREF, SIOD, SIOC, Y2-Y9). |
| *Bảo lưu: Octal PSRAM* | GPIO 35–37 | — | 8MB OPI PSRAM tốc độ cao cho xử lý khung hình tương lai. |
| *Bảo lưu: Khe thẻ nhớ SD* | GPIO 38–40 | — | Giao tiếp 1-bit SDMMC (CMD, CLK, DATA0). |
| *Bảo lưu: LED Onboard* | GPIO 2, 48 | — | LED chỉ thị và WS2812 RGB LED. |

### 2. Cổng Nạp & Console Log
- Cố định sử dụng cổng Type-C nhãn **UART** (bên trái bo mạch, qua chip nạp CH340 nối UART0 `GPIO43/44`).
- Cấu hình `CONFIG_ESP_CONSOLE_UART_DEFAULT=y` trong `sdkconfig.defaults` để đảm bảo kết nối Serial Monitor luôn ổn định qua các lần chip reboot/flash, không bị ngắt kết nối cổng COM như USB CDC.

### 3. Cấu Hình Phần Mềm & Bluetooth NimBLE
- Cập nhật target ESP-IDF: `idf.py set-target esp32s3`.
- Chuẩn hóa `sdkconfig.defaults`:
  - `CONFIG_BT_ENABLED=y`
  - `CONFIG_BT_BLUEDROID_ENABLED=n`
  - `CONFIG_BT_NIMBLE_ENABLED=y`
  - `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256`
  - Loại bỏ các cờ Bluetooth Classic (`BTDM`) vì phần cứng radio của ESP32-S3 thuần túy là BLE 5.0.

### 4. Tính Độc Lập Kiến Trúc & Không Ảnh Hưởng Hệ Thống Hiện Có
- **Firmware STM32 (`agv_stm32`)**: Không thay đổi bất kỳ dòng mã nào. Giao thức nhị phân `comms_protocol.c/.h` tương thích nhị phân 100%.
- **PC BLE Client (`agv_ble_controller.py`)**: Giữ nguyên toàn bộ 15 lệnh CLI và các tính năng chẩn đoán (`monitor_ir`, `calibrate_line`, `step`).
- **Phạm vi giai đoạn này**: Chưa khởi tạo driver Camera (dành riêng cho giai đoạn sau).

---

## Hệ quả & Đánh giá (Consequences)

### Tích Cực (Positive)
- **Sẵn sàng cho tương lai**: Nền tảng ESP32-S3 với 8MB Octal PSRAM và bus DVP camera được giải phóng hoàn toàn, sẵn sàng tích hợp thị giác máy tính mà không cần phải thay đổi lại sơ đồ nối dây UART với STM32.
- **Tốc độ và hiệu năng cao**: CPU Xtensa LX7 dual-core mạnh mẽ hơn, hỗ trợ tập lệnh vector AI, radio BLE 5.0 ổn định với MTU 256 byte.
- **Cách ly an toàn**: Chân GPIO 41 và 42 nằm ở hàng header bên ngoài, không phải strapping pin, không gây xung đột logic khi khởi động hoặc nạp code.
- **Tương thích toàn diện**: Toàn bộ 132 bài test tự động (22 test BLE client và 110 test STM32) đều chạy pass 100%.

### Tiêu Cực / Hạn Chế (Negative / Trade-offs)
- Phải dùng dây nối jumper vật lý vào chân GPIO 41 và 42 thay vì các chân mặc định cũ của ESP32 classic.
