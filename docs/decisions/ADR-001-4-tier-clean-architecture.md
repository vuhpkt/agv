# ADR-001: Phân tầng Kiến trúc 4 lớp và Loại bỏ thư mục Components

## Trạng thái (Status)
**Đã chấp thuận (Accepted)**

## Ngày quyết định (Date)
2026-09-06

## Bối cảnh (Context)
Ban đầu, mã nguồn dự án xe tự hành AGV STM32F103 có một số vấn đề thường gặp trong lập trình nhúng truyền thống:
1. Tầng ứng dụng (`app_main.c`, `app_motor_control.c`, `app_step_test.c`) gọi trực tiếp các API phần cứng của ST HAL (chẳng hạn `HAL_GetTick()`).
2. Mã ngắt phần cứng (`stm32f1xx_it.c`) chứa mã xử lý UART truyền nhận, phụ thuộc trực tiếp vào `bsp_uart.h`.
3. Tồn tại một thư mục trung gian `Components/` chứa lẫn lộn giữa IC driver phần cứng (`tb6612`), driver kênh debug (`SEGGER_RTT`), thuật toán điều khiển (`motor_pi`, `pid`), thuật toán lọc số (`filter`), mô hình động học robot (`kinematics`), và cấu trúc dữ liệu (`ring_buffer`). Điều này khiến danh sách include path trong `CMakeLists.txt` bị phình to (8 thư mục riêng lẻ) và ranh giới trách nhiệm giữa các lớp bị mờ nhạt.
4. Việc kiểm thử tự động (Unit Test) gặp khó khăn nếu logic nghiệp vụ dính chặt với phần cứng STM32.

## Quyết định (Decision)
Dự án quyết định tái cấu trúc toàn diện theo mô hình kiến trúc 4 tầng rõ ràng, chuẩn hóa theo nguyên lý Inversion of Control (IoC) và Clean Architecture:

```
┌────────────────────────────────────────────────────────┐
│ 1. Application Layer (app_main, app_comms, ...)        │
└───────────────────────────┬────────────────────────────┘
                            │ (gọi hàm API)
┌───────────────────────────▼────────────────────────────┐
│ 2. Services Layer (Zero-HAL, Pure C Algorithms)        │
│    velocity_service, comms_protocol, kinematics, ...   │
└───────────────────────────┬────────────────────────────┘
                            │ (gọi qua BSP Facade)
┌───────────────────────────▼────────────────────────────┐
│ 3. Hardware Adapter & Drivers                          │
│    - BSP: bsp_board, bsp_motor, bsp_encoder, bsp_uart  │
│    - Drivers: tb6612 (IC Driver), SEGGER_RTT (Debug)   │
└───────────────────────────┬────────────────────────────┘
                            │ (gọi ngoại vi)
┌───────────────────────────▼────────────────────────────┐
│ 4. Microcontroller Core & HAL (STM32CubeMX)            │
│    main.c, gpio.c, tim.c, usart.c, stm32f1xx_it.c     │
└────────────────────────────────────────────────────────┘
```

Chi tiết phân tách:
1. **Xóa hoàn toàn thư mục `Components/`**:
   - Chuyển IC driver động cơ sang `Drivers/tb6612/`.
   - Chuyển driver kênh debug sang `Drivers/SEGGER_RTT/`.
   - Chuyển toàn bộ các thuật toán, mô hình và cấu trúc dữ liệu thuần C (`encoder`, `filter`, `kinematics`, `motor_pi`, `pid`, `ring_buffer`) vào `Services/Inc/` và `Services/Src/`.
2. **Nguyên lý Dependency Inversion (DIP) cho IC Drivers**:
   - `tb6612` định nghĩa cấu trúc callback IO (`tb6612_io_t`) hoàn toàn độc lập với STM32 (Pure C, Zero-HAL).
   - Tầng `BSP` (`bsp_motor.c`) triển khai các hàm callback cụ thể bằng HAL và "tiêm" (inject) vào `tb6612`. Nhờ đó `tb6612` có thể test độc lập 100% trên PC.
3. **Trừu tượng hóa thời gian và ngắt**:
   - Tầng ứng dụng chỉ lấy thời gian qua `bsp_time_get_ms()`, tuyệt đối không gọi `HAL_GetTick()`.
   - Chuyển các hàm weak callback của UART (`HAL_UART_TxCpltCallback`, `HAL_UART_ErrorCallback`) về `bsp_uart.c`, giữ cho `stm32f1xx_it.c` sạch sẽ.
4. **Không cấp phát động (Zero Dynamic Memory)**:
   - 100% bộ nhớ trong `Services` và `BSP` được cấp phát tĩnh (static memory allocation).

## Các giải pháp thay thế đã cân nhắc (Alternatives Considered)

### Giữ nguyên cấu trúc thư mục `Components/`
- **Ưu điểm**: Không cần di chuyển file hay cập nhật `CMakeLists.txt`.
- **Nhược điểm**: Pha trộn giữa driver phần cứng (như TB6612, RTT) và thuật toán phần mềm (như PID, Kinematics). Khó giải thích ranh giới lớp cho kỹ sư mới. Số lượng include path trong CMake tăng đột biến theo số component.
- **Lý do bác bỏ**: Gây nhập nhằng về trách nhiệm và không tuân thủ quy chuẩn phân tầng nhúng hiện đại.

### Gộp chung tất cả vào thư mục `Core/`
- **Ưu điểm**: Ít thư mục, dễ tìm kiếm với dự án nhỏ.
- **Nhược điểm**: Mỗi lần CubeMX sinh lại mã nguồn (Generate Code), nguy cơ cao bị ghi đè. Logic nghiệp vụ bị dính chặt với HAL của STM32, không thể viết Unit Test tự động trên CI/CD x86.
- **Lý do bác bỏ**: Phá vỡ tính di động (portability) và khả năng kiểm thử tự động.

## Hệ quả (Consequences)
- **Tích cực**:
  - `CMakeLists.txt` tinh gọn: chỉ cần khai báo `Application/Inc`, `BSP/Inc`, `Services/Inc`, `Drivers/tb6612`, `Drivers/SEGGER_RTT`.
  - Bộ kiểm thử Python (`tests/`) có thể chạy độc lập không cần phần cứng, xác minh thuật toán PI, bộ lọc LPF, và giao thức CLI trong 1.4 giây.
  - Dự án sẵn sàng mở rộng thuật toán cấp cao hơn (như Odometry 2D, Pure Pursuit, ROS2 Twist controller) trong tầng `Services` và `Application` mà không lo ảnh hưởng phần cứng bên dưới.
- **Ràng buộc duy trì**:
  - Tầng `Services` và `Application` không được `#include` bất kỳ header nào của HAL (`stm32f1xx_hal.h`, `tim.h`, v.v.).
  - Mọi thao tác ngoại vi phải thông qua giao diện của tầng `BSP`.

