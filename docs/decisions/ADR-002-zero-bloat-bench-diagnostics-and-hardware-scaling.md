# ADR-002: Kiến trúc Chẩn đoán Bàn Test Zero-Bloat và Chuẩn hóa Tỷ lệ Xung PWM Tầng BSP

## Trạng thái (Status)
**Đã chấp thuận (Accepted)**

## Ngày quyết định (Date)
2026-09-08

## Bối cảnh (Context)
Trong quá trình triển khai và thử nghiệm thực tế xe tự hành AGV trên vi điều khiển STM32F103C8T6 (64 KB Flash, 20 KB RAM), dự án đối mặt với 3 thách thức kỹ thuật:

1. **Nguy cơ phình to mã nguồn (Firmware Bloat) do tính năng hiệu chuẩn**:
   - Mảng 5 mắt cảm biến quang học hồng ngoại cần hiệu chuẩn ngưỡng điện áp analog (nền trắng / vạch đen) và động cơ / encoder cần kiểm tra chiều quay thực tế trên bàn làm việc gần laptop (cáp nạp ST-Link ngắn).
   - Nếu nhúng các bộ phân tích cú pháp chuỗi ký tự CLI phức tạp, lưu trữ tham số vào Flash nội hoặc thuật toán hiệu chuẩn nặng nề vào luồng chính, firmware sản phẩm thực tế sẽ bị lãng phí bộ nhớ Flash/RAM và có nguy cơ gây trễ chu kỳ điều khiển vòng kín 100Hz (10ms).
2. **Lỗi sai lệch thang đo tỷ lệ băm xung PWM (Duty Cycle Scaling Bug)**:
   - Các thuật toán điều khiển cấp cao (`velocity_service`, `motor_pi`, `line_tracker`) và giao thức CLI sử dụng dải giá trị chuẩn hóa $u \in [0, 1000]$ (`BSP_MOTOR_MAX_SPEED = 1000`).
   - Trong khi đó, phần cứng Timer 4 được CubeMX cấu hình ở chu kỳ đếm $\text{ARR} = 3599$ (chu kỳ 3600 xung tại $72\text{ MHz}$) để tạo tần số PWM siêu âm $20\text{ kHz}$.
   - Việc ghi trực tiếp giá trị `duty` từ tầng trên vào thanh ghi `CCR` của Timer mà không quy đổi tỷ lệ khiến chu kỳ nhiệm vụ thực tế bị giảm đi 3.6 lần (lệnh 40% PWM chỉ cho ra 11.11% duty, điện áp nguồn $4.9\text{V}$ bị sụt xuống chỉ còn $0.54\text{V}$, khiến động cơ không đủ lực quay).
3. **Tính linh hoạt khi phát hiện ngược chiều động cơ hoặc encoder**:
   - Khi lắp ráp phần cứng, động cơ hoặc encoder có thể bị quay/đếm ngược chiều vật lý so với hướng quy ước tiến của xe. Nếu bắt buộc người dùng phải tháo lắp hàn lại dây hoặc sửa sơ đồ chân trong STM32CubeMX thì rất bất tiện.

---

## Quyết định (Decision)

### 1. Kiến trúc Chẩn đoán Bàn Test Tách biệt Hoàn toàn (Zero-Bloat Bench Diagnostics)
- **Chuyển toàn bộ gánh nặng xử lý sang phía máy tính (Host PC)**:
  - Viết các công cụ Python chuyên dụng (`scripts/calibrate_line.py`, `scripts/test_motor_direction.py`) chạy trên máy tính, kết nối với STM32 qua cổng OpenOCD RTT Server (TCP port 9090).
  - Máy tính chịu trách nhiệm: phân tích dải động, kiểm tra điều kiện $\Delta \ge 100$, đối chiếu tương quan chiều quay bánh xe / encoder, in bảng đánh giá trực quan và sinh trực tiếp file mã nguồn C [`Application/Inc/app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h).
- **Cơ chế cô lập bằng tiền xử lý (`#if AGV_ENABLE_MOTOR_TEST_MODE`) & Linker Garbage Collection**:
  - Khi bật chế độ bàn test (`AGV_ENABLE_MOTOR_TEST_MODE = 1`), các module chẩn đoán (`diag_motor`, `diag_ir`, `app_comms`) hoạt động phục vụ gỡ lỗi.
  - Khi nạp firmware chạy thực tế trên sàn thi đấu (`AGV_ENABLE_MOTOR_TEST_MODE = 0`), bộ tiền xử lý C loại bỏ 100% các lời gọi hàm test trong vòng lặp chính. Bộ liên kết GCC với cờ `-Wl,--gc-sections` sẽ tự động bóc tách hoàn toàn các hàm không dùng ra khỏi file nhị phân, đảm bảo **0 byte Flash rác, 0 byte RAM rác và 0% trễ chu kỳ điều khiển thực tế**.

### 2. Chuẩn hóa Ánh xạ Tỷ lệ PWM Động tại Tầng BSP
- Tầng Thuật toán (`Services`) và Tầng Ứng dụng (`Application`) tiếp tục duy trì dải điều khiển chuẩn hóa $[0, \text{BSP\_MOTOR\_MAX\_SPEED}]$ với giá trị $1000$ (tương ứng 100% công suất).
- Tầng Hardware Adapter [`BSP/Src/bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c) thực hiện ánh xạ tỷ lệ động:
  ```c
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);
  uint32_t pulse = ((uint32_t)duty * period) / BSP_MOTOR_MAX_SPEED;
  if (pulse > period) pulse = period;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint16_t)pulse);
  ```
  Nhờ sử dụng `__HAL_TIM_GET_AUTORELOAD()`, mã nguồn tự động tương thích kể cả khi người dùng thay đổi tần số PWM hoặc chu kỳ Period trong STM32CubeMX mà không cần sửa code C.

### 3. Tích hợp Cờ Cấu hình Chiều Quay Phần mềm (`inverted`)
- Module `tb6612` và `encoder` được trang bị cờ cấu hình boolean `.inverted`:
  ```c
  /* Đảo chiều động cơ trái 100% bằng phần mềm */
  tb6612_cfg_t cfg_left = {
      .max_pwm = BSP_MOTOR_MAX_SPEED,
      .inverted = true
  };
  ```
- Khi phát hiện động cơ hoặc encoder quay ngược chiều xe tiến, chỉ cần đổi `false` thành `true` trong tầng BSP (`bsp_motor.c` hoặc `bsp_encoder.c`) mà không cần can thiệp phần cứng hay sửa sơ đồ chân CubeMX.

---

## Các Giải pháp Thay thế Đã Cân Nhắc (Alternatives Considered)

### 1. Nhúng Toàn bộ Trình Calib & Lưu EEPROM trên STM32
- **Lý do bác bỏ**: Làm tăng dung lượng Flash thêm ít nhất 8-12 KB cho bộ parser chuỗi và driver Flash Emulated EEPROM. STM32F103 không có EEPROM phần cứng, việc ghi Flash nội trong lúc xe đang chạy tiềm ẩn rủi ro treo CPU hoặc làm mòn sector Flash.

### 2. Đổi `BSP_MOTOR_MAX_SPEED` từ 1000 thành 3599
- **Lý do bác bỏ**: Khiến các thuật toán thuần C trong tầng `Services` bị phụ thuộc cứng vào cấu hình phần cứng của vi điều khiển (vi phạm nguyên tắc Zero-HAL Clean Architecture). Nếu sau này chuyển sang vi điều khiển khác hoặc đổi tần số PWM, toàn bộ thuật toán PI, Feedforward, Deadzone và các bài test Python đều phải viết lại. Ánh xạ tại tầng BSP là phương án chuẩn mực nhất.

### 3. Bắt buộc Người Dùng Đổi Cực Dây Động Cơ bằng Phần Cứng
- **Lý do bác bỏ**: Gây bất tiện lớn cho người dùng, đặc biệt khi mạch đã được hàn chết hoặc đi dây thẩm mỹ trong khung gầm xe. Xử lý bằng cờ logic `.inverted` ở tầng driver giải quyết triệt để vấn đề chỉ trong vài giây.

---

## Hệ Quả (Consequences)
- **Tích cực**:
  - Firmware chạy thực tế giữ nguyên chuẩn Zero-Bloat tối đa.
  - Khắc phục hoàn toàn lỗi sụt áp 0.5V, động cơ đáp ứng chuẩn xác 100% công suất theo điện áp nguồn.
  - Hệ thống sở hữu bộ công cụ kiểm thử tương tác chuyên nghiệp trên PC, giúp người dùng chẩn đoán nhanh chóng lỗi lệch chiều và dải động cảm biến.
  - Tầng dịch vụ thuần C hoàn toàn độc lập với thông số Timer phần cứng.
- **Tiêu cực / Ràng buộc**:
  - Khi đổi điện áp nguồn $V_m$ hoặc thay động cơ mới, người dùng cần chạy lệnh `MOT A 1000 2500` qua RTT để cập nhật lại hệ số Feedforward $K_{ff}$ tối ưu trong file `motor_pi.h`.

