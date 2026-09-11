# Sổ tay Xử lý Sự cố & Câu hỏi Thường gặp (Troubleshooting & FAQ)

> **Mục đích**: Tài liệu này được thiết kế để bạn sau 6 tháng quay lại, hoặc một kỹ sư mới tiếp cận dự án, có thể tự mình giải quyết 99% các tình huống lỗi phần cứng, mạch nạp, công cụ Python và thuật toán mà không cần phải lục lại toàn bộ commit history.

---

## 🧭 Lộ trình 30 phút Tiếp cận Dự án (Onboarding Pathway)

Dành cho người mới bắt đầu hoặc sau thời gian dài không chạm vào code:

```
[Phút 00 - 10] TỔNG QUAN HỆ THỐNG
  ├── Đọc README.md (Mục lục, yêu cầu toolchain, phím tắt CLI)
  └── Đọc docs/architecture.md (Hiểu kiến trúc 4 tầng: Drivers -> BSP -> Services -> App)

[Phút 10 - 20] PHẦN CỨNG & CẤU HÌNH
  ├── Xem docs/hardware_and_pinout.md (Bảng chân vi điều khiển, TB6612, Encoder, 5 mắt IR)
  └── Xem docs/configuration_guide.md (Bảng tra cứu tham số trong code)

[Phút 20 - 30] THỰC HÀNH TEST BÀN THỬ NGHIỆM
  ├── Mở Terminal 1: ./rtt.sh (Bật OpenOCD RTT Server port 9090)
  ├── Mở Terminal 2: py scripts/test_motor_direction.py (Test quay tiến/lùi 2 bánh)
  └── Mở Terminal 2: py scripts/monitor_ir.py --track (Xem lệch vạch và mắt IR)
```

---

## 🛠️ 1. Sự cố Mạch nạp & Debug (ST-Link & OpenOCD)

### 1.1. Lỗi: `Error: init mode failed (unable to connect to the target)`
* **Triệu chứng**: Chạy `./flash.sh` hoặc `./rtt.sh` báo không tìm thấy chip STM32 hoặc ST-Link từ chối kết nối.
* **Nguyên nhân**:
  1. Cáp nối SWD (SWDIO: PA13, SWCLK: PA14, GND, 3V3) bị lỏng hoặc cắm nhầm chân.
  2. Vi điều khiển đang ở chế độ Stop/Standby tiết kiệm pin hoặc các chân SWD bị firmware cũ cấu hình lại thành GPIO thông thường.
  3. Cáp USB của ST-Link tiếp xúc kém.
* **Cách khắc phục**:
  - Kiểm tra lại 4 dây SWD (đặc biệt là dây GND phải chung).
  - **Khôi phục bằng chân BOOT0**:
    1. Gạt jumper `BOOT0` trên mạch BluePill từ `0` sang `1`.
    2. Nhấn và giữ nút `RESET` trên STM32.
    3. Chạy lệnh nạp: `./flash.sh`.
    4. Ngay khi OpenOCD bắt đầu nhận dạng ST-Link, thả nút `RESET`.
    5. Sau khi nạp xong thành công, gạt lại jumper `BOOT0` về `0`.

### 1.2. Lỗi: Chân PA15, PB3, PB4 không điều khiển được động cơ
* **Nguyên nhân**: STM32F103 mặc định dùng các chân này cho chuẩn gỡ lỗi JTAG.
* **Đã xử lý trong code**: Hàm `BSP_Motor_Init()` tại [`bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c) đã gọi `__HAL_AFIO_REMAP_SWJ_NOJTAG()`. Lệnh này vô hiệu hóa JTAG nhưng vẫn giữ nguyên SWD (PA13/PA14).
* **Lưu ý sống còn**: Tuyệt đối không gọi `__HAL_AFIO_REMAP_SWJ_DISABLE()`, nếu không bạn sẽ bị khóa cổng SWD vĩnh viễn và bắt buộc phải dùng BOOT0 để nạp lại qua UART!

---

## 🚗 2. Sự cố Động cơ & Encoder (Motor & Encoders)

### 2.1. Động cơ quay lùi khi xe muốn đi tiến, hoặc 2 bánh quay ngược chiều nhau (xe quay tròn)
* **Nguyên nhân**: Đấu dây động cơ DC bị đảo cực hoặc khung gầm lắp 2 động cơ đối xứng ngược chiều.
* **Cách khắc phục nhanh (không cần hàn lại dây)**:
  - Mở [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c):
    - Tìm cấu hình `cfg_left.inverted` hoặc `cfg_right.inverted`.
    - Đổi giữa `true` và `false`.
  - Mở [`BSP/Src/bsp_encoder.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_encoder.c):
    - Đồng bộ cờ `cfg_left.inverted` hoặc `cfg_right.inverted` của encoder tương ứng.
  - Chạy lại kiểm tra tự động:
    ```bash
    py scripts/test_motor_direction.py
    ```
  - Chi tiết ma trận 4 trường hợp xem tại [`docs/motor_encoder_direction_guide.md`](motor_encoder_direction_guide.md).

### 2.2. Động cơ kêu rít nhưng không quay ở tốc độ thấp
* **Nguyên nhân**: Ma sát tĩnh cơ khí hộp số lớn hơn mô-men khởi động (Vùng chết - Deadband).
* **Cách khắc phục**:
  - Mở [`Application/Src/app_motor_control.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Src/app_motor_control.c) hoặc tinh chỉnh qua struct `MotorPIConfig`:
    - Tăng `deadzone_pwm` (mặc định ~100-150 / 1000). Bộ điều khiển sẽ tự động cộng thêm điện áp vượt qua vùng chết ma sát khi bắt đầu quay.

### 2.3. Bánh xe bị giật cục hoặc vọt lố tốc độ (Hunting / Overshoot)
* **Nguyên nhân**: Hệ số $K_p$ hoặc $K_i$ quá lớn, hoặc chưa nhận dạng đúng hàm truyền động cơ.
* **Cách khắc phục**:
  - Chạy công cụ nhận dạng bước nhảy:
    ```bash
    py scripts/plot_step_response.py --ch 0 --duty 800 --time 1500
    ```
  - Tính lại hệ số $K_{ff} = \frac{1}{\text{RPM}_{\max}}$, $K_p = \frac{\tau}{K \cdot \tau_{cl}}$.
  - Mở [`docs/control_theory.md`](control_theory.md) để tra bảng công thức pole-placement.

---

## 👁️ 3. Sự cố Mảng Cảm biến 5 Mắt IR (Optical Line Sensor)

### 3.1. Cảm biến đọc giá trị ADC cực thấp (~16 hoặc ~0) hoặc không phản hồi
* **Nguyên nhân**:
  1. Dây cấp nguồn 5V/3.3V hoặc GND của thanh cảm biến IR bị lỏng.
  2. Mạch phát LED hồng ngoại chưa sáng (kiểm tra bằng camera điện thoại xem các mắt phát có sáng tím mờ không).
  3. Thứ tự chân analog bị đảo.
* **Thứ tự đúng của mảng IR trên xe**:
  - Kênh 0: `PA2` (R2 - Ngoài cùng bên phải)
  - Kênh 1: `PA3` (R1 - Phải)
  - Kênh 2: `PA4` (MID - Tâm giữa)
  - Kênh 3: `PA5` (L1 - Trái)
  - Kênh 4: `PB0` (L2 - Ngoài cùng bên trái)
* **Kiểm tra trực tiếp qua Terminal RTT**:
  ```bash
  # Khởi động OpenOCD RTT server
  ./rtt.sh
  # Terminal khác: Xem giá trị ADC thô của 5 mắt
  py scripts/monitor_ir.py --raw
  ```

### 3.2. Hiệu chuẩn xong nhưng xe vẫn nhận diện sai vạch
* **Nguyên nhân**: Thay đổi ánh sáng môi trường (ánh đèn tuýp, ánh nắng) hoặc chiều cao thanh cảm biến so với mặt sàn bị lệch.
* **Cách khắc phục chuẩn**:
  1. Đặt xe trên đúng mặt sàn thi đấu.
  2. Chạy script quét dải động:
     ```bash
     py scripts/calibrate_line.py --scan 5000
     ```
  3. Lia xe qua lại ngang vạch trong 5 giây. Script sẽ in bảng Min/Max và hỏi có lưu vào [`Application/Inc/app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h) không.
  4. Chọn `Y` (Yes) rồi biên dịch lại `./build.sh` và nạp `./flash.sh`.

---

## 💻 4. Sự cố Môi trường & Hệ điều hành (Git, Python, Windows)

### 4.1. Lỗi Git: `fatal: .git/index: index file smaller than expected`
* **Nguyên nhân**: Thao tác tắt đột ngột máy tính trên Windows hoặc xung đột cache làm hỏng file `.git/index`.
* **Cách khắc phục 1-click**:
  - Chạy file script đã được chuẩn bị sẵn:
    ```cmd
    scripts\fix_git_index.bat
    ```
  - Script sẽ tự động xóa index hỏng và khôi phục lại trạng thái git mà không làm mất bất kỳ dòng code nào.

### 4.2. Lỗi Python: `ModuleNotFoundError: No module named 'numpy'`
* **Nguyên nhân**: Trên Windows có nhiều môi trường Python khác nhau (ví dụ: Python của MSYS2 vs Python cài ngoài).
* **Cách khắc phục**:
  - Sử dụng launcher `py -3`:
    ```bash
    py -3 -m pip install numpy matplotlib
    py -3 -m unittest discover tests
    ```

---

## ⚡ 5. Tóm tắt Kiểm tra Trước khi Đặt Xe Xuống Sàn (Pre-Flight Checklist)

Trước mỗi lần cho AGV chạy tự hành dưới sàn, hãy làm theo danh sách kiểm tra nhanh 5 bước sau:

- [ ] **1. Điện áp pin**: Đo pin LiPo $\ge 7.4\text{V}$ (hoặc $\ge 11.1\text{V}$ tùy cấu hình pin). Pin yếu sẽ làm tụt tốc độ cực đại và làm sai lệch Feedforward $K_{ff}$.
- [ ] **2. Kiểm tra chế độ vận hành trong [`Application/Inc/app_config.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_config.h)**:
  - Nếu muốn tự chạy khi bật công tắc: `#define AGV_LINE_FOLLOW_AUTO_START 1`
  - Nếu chuẩn bị thi đấu: `#define AGV_ENABLE_MOTOR_TEST_MODE 0` (để Linker thu gọn Flash về 42.5 KB).
- [ ] **3. Chiều cao cảm biến IR**: Khoảng cách từ mắt phát/thu IR xuống sàn đạt chuẩn từ **5mm - 10mm**.
- [ ] **4. Bánh xe & Encoder**: Dùng tay xoay nhẹ 2 bánh xem có bị kẹt rác hoặc hộp số bị vướng không.
- [ ] **5. Đặt xe lên vạch**: Đặt mắt giữa (MID - CH2) trùng vào vạch đen, bật công tắc nguồn. Đèn LED PC13 sẽ nháy nhanh 100ms trong 3 giây đếm ngược an toàn rồi xe xuất phát.

