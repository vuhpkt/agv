# Quy tắc và Chỉ dẫn Dự án AGV STM32 (agv_stm32)

## 1. Môi trường & Toolchain
- **Vi điều khiển**: STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)
- **Trình biên dịch**: `arm-none-eabi-gcc` (C11)
- **Hệ thống Build**: CMake + Ninja (Thư mục build: `build/Debug`)
- **Mạch nạp & Debug**: ST-Link V2 qua OpenOCD

## 2. Các lệnh thao tác chính (Git Bash / Shell)
Mỗi khi cần biên dịch hoặc nạp firmware, hãy ưu tiên dùng các script hoặc lệnh sau:
- **Biên dịch**: `./build.sh` (hoặc `cmake --build build/Debug`)
- **Nạp Firmware**: `./flash.sh` (hoặc `cmake --build build/Debug -t flash`)
- **Giao tiếp UART1 (với ESP32)**: Chân PA9 (TX), PA10 (RX), Baudrate 115200 8N1 qua DMA Circular + RingBuffer.

## 3. Kiến trúc dự án
- `Application/`: Logic ứng dụng cấp cao AGV, máy trạng thái chuỗi lộ trình FSM, giao tiếp UART nhị phân (`app_main.c`, `app_comms.c`, `app_motor_control.c`, `app_line_follow.c`).
- `Diagnostics/`: Tầng chẩn đoán phần cứng và tự kiểm tra độc lập từng thiết bị (`diag_motor.c`, `diag_ir.c`).
- `Services/`: Tầng Dịch vụ & Thuật toán thuần C (Zero-HAL, 100% static memory):
  - `line_tracker`: Thuật toán bám line, chuẩn hóa quang học, tính sai số trọng tâm e_line, Line PID.
  - `velocity_service`: Dịch vụ điều khiển vận tốc 2 kênh động cơ (tích hợp LPF, CPR, Motor PI).
  - `comms_protocol`: Bộ phân tích và đóng gói giao thức nhị phân MCU-to-MCU (Header 0xAA 0x55, CRC16-CCITT).
  - `encoder`: Giải thuật xử lý xung, delta từng chu kỳ, bù tràn số Timer 16-bit và tổng xung 32-bit.
  - `filter`: Bộ lọc số thông thấp bậc 1 (LPF / EMA).
  - `kinematics`: Động học vi sai AGV và tính toán Odometry 2D (x, y, theta).
  - `motor_pi`: Bộ điều khiển PI vận tốc động cơ có bù ma sát (Feedforward + Slew-rate).
  - `pid`: Thuật toán PID Controller với Anti-windup.
  - `ring_buffer`: Hàng đợi vòng FIFO phục vụ truyền nhận UART DMA.
- `BSP/`: Tầng Hardware Adapter / Binding — liên kết trực tiếp phần cứng STM32 (GPIO, PWM, Counter, DMA, ADC, UART), triển khai callbacks I/O tiêm vào Drivers (`bsp_uart.c`, `bsp_motor.c`, `bsp_encoder.c`, `bsp_board.c`, `bsp_ir.c`).
- `Drivers/`: Driver phần cứng ngoại vi, IC và vi điều khiển:
  - `tb6612/`: Driver điều khiển IC cầu H TB6612FNG (bảng chân lý, chiều quay, phanh, duty PWM).
  - `CMSIS/`: ARM Cortex-M3 Core Drivers.
  - `STM32F1xx_HAL_Driver/`: ST HAL Drivers.
- `Core/`: Cấu hình vi điều khiển & ngoại vi STM32CubeMX HAL (`main.c`, `tim.c`, `usart.c`, `dma.c`, `adc.c`, `gpio.c`, `stm32f1xx_it.c`).

