# Danh Sách Nhiệm Vụ Triển Khai: Chuyển Đổi Sang Freenove ESP32-S3 CAM

## Task 1: Cấu hình ESP-IDF Target `esp32s3` & Chuẩn hóa `sdkconfig.defaults`
**Description:** Chuyển target của dự án `agv_esp32` sang vi điều khiển `esp32s3` trên ESP-IDF v6.0.2. Tinh chỉnh `sdkconfig.defaults` loại bỏ các cờ BT classic không hỗ trợ trên S3, kích hoạt NimBLE và ATT MTU 256.

**Acceptance criteria:**
- [x] `sdkconfig.defaults` chứa đầy đủ các cấu hình NimBLE BLE cho S3 (`CONFIG_BT_ENABLED=y`, `CONFIG_BT_BLUEDROID_ENABLED=n`, `CONFIG_BT_NIMBLE_ENABLED=y`, `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256`).
- [x] `idf.py set-target esp32s3` chạy trơn tru, sinh file `sdkconfig` nhắm đúng kiến trúc Xtensa LX7 của ESP32-S3.

**Verification:**
- [x] Lệnh: `idf.py set-target esp32s3` thực thi thành công không lỗi trong môi trường ESP-IDF v6.x.

**Dependencies:** Không.

**Files touched:**
- `agv_esp32/sdkconfig.defaults`
- `agv_esp32/sdkconfig`

**Estimated scope:** Small (1-2 files).

---

## Task 2: Cập nhật Pinout UART (GPIO 42 TX, GPIO 41 RX) Trong `uart_service` & `main.c`
**Description:** Thay thế chân UART cũ (GPIO 16/17 đã bị dính bus camera) bằng cặp chân mới đã thống nhất: GPIO 42 (TX) và GPIO 41 (RX). Sử dụng `UART_NUM_1`, kết nối nút BOOT ở `GPIO_NUM_0`.

**Acceptance criteria:**
- [x] `main.c` định nghĩa `AGV_PIN_UART_TX = GPIO_NUM_42`, `AGV_PIN_UART_RX = GPIO_NUM_41`, `AGV_UART_PORT = UART_NUM_1`.
- [x] `uart_service.h` cập nhật macro mặc định `UART_SERVICE_CONFIG_DEFAULT()` với TX=42, RX=41, Port=UART1.
- [x] Nút BOOT vẫn giữ nguyên `AGV_PIN_BOOT_BTN = GPIO_NUM_0`.

**Verification:**
- [x] Build thành công: `idf.py build` sinh ra file `build/agv_esp32.bin` và `build/agv_esp32.elf`.

**Dependencies:** Task 1.

**Files touched:**
- `agv_esp32/main/main.c`
- `agv_esp32/components/uart_service/include/uart_service.h`

**Estimated scope:** Small (2 files).

---

## Checkpoint 1: Sau Task 1 & Task 2 (Biên Dịch Sạch Sẽ Cho ESP32-S3)
- [x] Firmware `agv_esp32` biên dịch thành công 100% cho kiến trúc `esp32s3`.
- [x] Không có cảnh báo hoặc lỗi liên quan đến NimBLE hay UART driver.

---

## Task 3: Cập nhật Tài Liệu Kỹ Thuật & Sơ Đồ Đấu Nối Freenove S3 CAM
**Description:** Bổ sung bảng chân cắm phần cứng chi tiết của kit Freenove ESP32-S3 CAM vào tài liệu đặc tả `SPEC-agv-esp32-binary-gateway.md` và `README.md`, giải thích rõ các chân đã dùng và các chân được bảo lưu cho Camera/PSRAM/SD.

**Acceptance criteria:**
- [x] `docs/specs/SPEC-agv-esp32-binary-gateway.md` có bảng đấu nối chi tiết giữa STM32F103 (PA9/PA10) và Freenove ESP32-S3 CAM (GPIO 41/42).
- [x] Ghi chú rõ ràng lý do tránh các chân GPIO 4–18 (Camera), GPIO 35–37 (PSRAM), GPIO 38–40 (SD Card).
- [x] `README.md` cập nhật thông số gateway là Freenove ESP32-S3 CAM.

**Verification:**
- [x] Kiểm tra nội dung markdown và liên kết tài liệu hiển thị chuẩn xác.

**Dependencies:** Task 2.

**Files touched:**
- `docs/specs/SPEC-agv-esp32-binary-gateway.md`
- `README.md`

**Estimated scope:** Small (2 files).

---

## Task 4: Kiểm Thử Toàn Diện Bộ Giao Thức & Client Python
**Description:** Chạy toàn bộ bộ test tự động của phía PC Python Client (`test_ble_controller.py`) và bộ test giao thức nhị phân của STM32 để đảm bảo việc di chuyển phần cứng sang S3 không làm phá vỡ logic truyền thông và chẩn đoán.

**Acceptance criteria:**
- [x] 22/22 unit tests của `test_ble_controller.py` vượt qua (parse log, telemetry line, monitor_ir, calibrate_line, step recorder).
- [x] 110/110 unit tests của STM32 vượt qua.

**Verification:**
- [x] Chạy: `py -3 -m unittest agv_esp32/tools/test_ble_controller.py` -> OK (Ran 22 tests in 0.001s).
- [x] Chạy: `cd agv_stm32 && py -3 -m unittest discover tests` -> OK (Ran 110 tests in 1.876s).

**Dependencies:** Task 2.

**Files touched:**
- `agv_esp32/tools/test_ble_controller.py`

**Estimated scope:** Extra Small (1 file kiểm thử).

---

## Checkpoint 2: Hoàn Tất Triển Khai & Bàn Giao
- [x] Tất cả các bài test phần mềm (132 tests tổng cộng) vượt qua.
- [x] Firmware binary `agv_esp32.bin` sẵn sàng nạp qua cổng Type-C "UART".
- [x] Bảng hướng dẫn cắm dây vật lý rõ ràng cho người dùng.
