# Cẩm Nang Phối Hợp & Đóng Góp Dự Án (Team Collaboration & Contributing Guide)

Chào mừng các thành viên trong nhóm tham gia phát triển dự án **AGV Autonomous Mobile Robot**! Tài liệu này quy định các chuẩn mực kiến trúc, quy trình làm việc trên Git và cách thức kiểm thử để đảm bảo hệ thống luôn ổn định, tin cậy và triển khai an toàn.

---

## 1. Yêu Cầu Môi Trường Phát Triển (Development Setup)

### 1.1. Công Cụ Lập Trình & Giao Diện Máy Tính (Host Tools)
- **Python**: Phiên bản 3.10 trở lên.
- Cài đặt toàn bộ thư viện cần thiết:
  ```bash
  pip install -r requirements.txt
  ```
- **IDE Khuyên Dùng**: Visual Studio Code.
  - Mở workspace bằng cách chọn: `File -> Open Workspace from File... -> AGV.code-workspace`.
  - Cài đặt các extension được khuyến nghị trong file workspace: *C/C++*, *CMake Tools*, *ESP-IDF*.

### 1.2. Chuỗi Công Cụ Biên Dịch Firmware
1. **STM32 Motion Controller (`agv_stm32`)**:
   - `arm-none-eabi-gcc` (phiên bản 10.3 hoặc 13.3+).
   - `CMake` (>= 3.22) & `Ninja`.
   - `OpenOCD` để nạp qua ST-Link V2.
2. **ESP32-S3 Gateway (`agv_esp32`)**:
   - `ESP-IDF v6.x` (khuyên dùng v6.0.2).
   - Cổng USB Type-C "UART" (bên trái trên kit Freenove ESP32-S3 CAM) để nạp code và debug.

---

## 2. Nguyên Tắc Thiết Kế Bất Biến (Architecture Invariants)

Mọi thay đổi mã nguồn firmware trên xe phải tuân thủ nghiêm ngặt các nguyên tắc kỹ thuật sau:

1. **Zero-Malloc / 100% Bộ Nhớ Tĩnh**:
   - Tuyệt đối **KHÔNG** dùng cấp phát bộ nhớ động (`malloc`, `calloc`, `free`) trên vi điều khiển STM32.
   - Mọi biến, mảng, hàng đợi lộ trình và bộ đệm UART DMA phải được cấp phát tĩnh (`static`) ở thời điểm biên dịch để triệt tiêu hoàn toàn rủi ro phân mảnh bộ nhớ (heap fragmentation).
2. **Kiến Trúc Phân Tầng Clean Architecture (STM32)**:
   - **Tầng Services (`Services/`)**: Chứa thuật toán điều khiển thuần C (`pure C`), độc lập 100% với phần cứng. **Cấm tuyệt đối include STM32 HAL** trong tầng Services.
   - **Tầng BSP & Drivers**: Đóng gói giao tiếp vi điều khiển (ADC, Timer, UART). Giao tiếp với tầng Services thông qua các hàm callback hoặc con trỏ hàm theo mẫu *Inversion of Control (IoC)*.
3. **Toàn Vẹn Dữ Liệu Giao Thức (Binary Protocol)**:
   - Giao tiếp giữa ESP32 và STM32 dùng khung truyền nhị phân có mã kiểm tra toàn vẹn `CRC16-CCITT` (0x1021). Bất kỳ lệnh điều khiển nào bị lỗi CRC đều bị loại bỏ ngay lập tức ở tầng thấp.
4. **Bảo Lưu Chân Phần Cứng Freenove ESP32-S3 CAM**:
   - UART1 kết nối STM32 cố định tại **GPIO 42 (TX) & GPIO 41 (RX)**.
   - Không được cấu hình chiếm dụng các chân GPIO 4–18 (Camera DVP) và GPIO 35–37 (Octal PSRAM 8MB).

---

## 3. Quy Trình Kiểm Thử Bắt Buộc Trước Khi Commit (Quality Gate)

Trước khi tạo commit hoặc mở Pull Request, bạn **bắt buộc** phải chạy kiểm thử tự động trên máy tính để đảm bảo không gây hồi quy (regression):

```bash
# 1. Chạy 120 bài unit test thuật toán STM32 (Line PID, Motor PI, Route Queue, Protocol)
py -3 -m unittest discover agv_stm32/tests

# 2. Chạy 36 bài unit test công cụ ESP32 / GUI Dashboard / Telemetry
py -3 -m unittest discover agv_esp32/tools

# 3. Kiểm tra biên dịch firmware STM32
cd agv_stm32
cmake --build build/Debug
cd ..
```

> **Tiêu chí nghiệm thu**: 100% các bài test (156/156) phải đạt trạng thái **OK** và không có lỗi biên dịch.

---

## 4. Quy Chuẩn Git Workflow & Commit (Conventional Commits)

Dự án áp dụng mô hình **Trunk-Based Development** với các nhánh tính năng ngắn ngày:

### 4.1. Quy Ước Đặt Tên Nhánh
- `feature/<tên-tính-năng>`: Thêm tính năng mới (ví dụ: `feature/obstacle-lidar`, `feature/auto-docking`).
- `fix/<mô-tả-lỗi>`: Sửa lỗi (ví dụ: `fix/motor-stall-recovery`).
- `docs/<chủ-đề>`: Cập nhật tài liệu kỹ thuật hoặc đặc tả.

### 4.2. Định Dạng Commit Message
Mỗi commit phải mang tính nguyên tử (atomic - giải quyết một việc cụ thể) và theo cấu trúc:
```text
<loại>(<phạm vi>): <mô tả ngắn gọn bằng tiếng Anh/Việt>

[Phần thân giải thích lý do (Why) và cách thức giải quyết, nếu cần]
```

**Các loại commit hợp lệ:**
- `feat`: Tính năng mới cho firmware, GUI hoặc công cụ.
- `fix`: Sửa lỗi thuật toán, giao tiếp hoặc bộ đệm.
- `docs`: Cập nhật tài liệu, specs hoặc ADRs.
- `refactor`: Tái cấu trúc mã nguồn mà không làm thay đổi hành vi hoạt động.
- `test`: Bổ sung hoặc cập nhật test cases.
- `chore`: Cấu hình build, dependencies hoặc scripts hỗ trợ.

---

## 5. Tài Liệu Tham Khảo Nhanh Trong Repo
- **Bức tranh tổng thể hệ thống**: [`docs/agv_system_overview_for_esp32.md`](docs/agv_system_overview_for_esp32.md)
- **Giao thức nhị phân UART**: [`docs/esp32_uart_protocol_guide.md`](docs/esp32_uart_protocol_guide.md)
- **Sơ đồ chân vi điều khiển**: [`docs/hardware_and_pinout.md`](docs/hardware_and_pinout.md)
- **Cẩm nang xử lý sự cố & FAQ**: [`docs/troubleshooting_and_faq.md`](docs/troubleshooting_and_faq.md)
