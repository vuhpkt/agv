# Sơ đồ Phần cứng & Đấu nối Chân (Hardware & Pinout)

Tài liệu này cung cấp bảng tra cứu toàn bộ sơ đồ chân, cấu hình ngoại vi Timer/PWM/Encoder và nguyên lý kết nối phần cứng của vi điều khiển **STM32F103C8T6** với IC điều khiển động cơ **TB6612FNG** và các cảm biến đọc xung Encoder.

---

## 1. Thông số Kỹ thuật Phần cứng

| Thành phần | Thông số | Ghi chú |
| :--- | :--- | :--- |
| **Vi điều khiển (MCU)** | STM32F103C8T6 (ARM Cortex-M3) | Xung nhịp hệ thống 72 MHz (HSE 8MHz + PLL 9x) |
| **Bộ nhớ** | 64 KB Flash, 20 KB SRAM | |
| **Driver Động cơ** | Toshiba TB6612FNG | Cầu H đôi MOSFET, dòng đỉnh 3.2A, liên tục 1.2A |
| **Tần số PWM Động cơ** | 20 kHz (TIM4, Prescaler=0, Period=3599) | Tần số siêu âm, êm ái, không gây rít cuộn dây |
| **Đọc xung Encoder** | Hardware Timer Encoder Mode 4X (TIM2, TIM3) | Đọc tự động phần cứng không tốn CPU ngắt |
| **Giao tiếp Lệnh & Log** | SEGGER RTT (SWD) + USART1 (TTL DMA) | Tốc độ cao, không nghẽn luồng điều khiển |

---

## 2. Bảng Phân bổ Chân Vi điều khiển (Pinout Table)

| Pin MCU | Tên Nhãn STM32 | Ngoại vi / Mode | Hướng I/O | Thiết bị kết nối | Chức năng chi tiết |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **PC13** | `LED_STATUS` | GPIO Output PP | Ra | Onboard LED | Báo hiệu Heartbeat (Active LOW: mức 0 sáng, mức 1 tắt) |
| **PA0** | `ENCA_CH_1` | TIM2_CH1 | Vào | Encoder Động cơ Trái (Kênh A) | Đọc xung cầu vuông Kênh 1 |
| **PA1** | `ENCA_CH_2` | TIM2_CH2 | Vào | Encoder Động cơ Trái (Kênh B) | Đọc xung cầu vuông Kênh 2 |
| **PA2** | `IR_IN1` | ADC1_IN2 (DMA1 Ch1) | Vào | Cảm biến IR Kênh 1 (Ngoài cùng Phải - R2) | Điện áp analog phản xạ bề mặt |
| **PA3** | `IR_IN2` | ADC1_IN3 (DMA1 Ch1) | Vào | Cảm biến IR Kênh 2 (Phải - R1) | Điện áp analog phản xạ bề mặt |
| **PA4** | `IR_IN3` | ADC1_IN4 (DMA1 Ch1) | Vào | Cảm biến IR Kênh 3 (Tâm - MID) | Điện áp analog phản xạ bề mặt |
| **PA5** | `IR_IN4` | ADC1_IN5 (DMA1 Ch1) | Vào | Cảm biến IR Kênh 4 (Trái - L1) | Điện áp analog phản xạ bề mặt |
| **PA6** | `ENCB_CH1` | TIM3_CH1 | Vào | Encoder Động cơ Phải (Kênh A) | Đọc xung cầu vuông Kênh 1 |
| **PA7** | `ENCB_CH2` | TIM3_CH2 | Vào | Encoder Động cơ Phải (Kênh B) | Đọc xung cầu vuông Kênh 2 |
| **PB0** | `IR_IN5` | ADC1_IN8 (DMA1 Ch1) | Vào | Cảm biến IR Kênh 5 (Ngoài cùng Trái - L2) | Điện áp analog phản xạ bề mặt |
| **PA9** | `USART1_TX` | USART1_TX (DMA1 Ch4) | Ra | Cổng Serial / USB-UART | Truyền dữ liệu telemetry / phản hồi CLI |
| **PA10** | `USART1_RX` | USART1_RX (DMA1 Ch5) | Vào | Cổng Serial / USB-UART | Nhận lệnh điều khiển từ PC / Raspberry Pi |
| **PA13** | `SWDIO` | SYS_JTMS-SWDIO | I/O | Mạch nạp ST-Link V2 | Nạp firmware và giao tiếp RTT (SWD) |
| **PA14** | `SWCLK` | SYS_JTCK-SWCLK | Vào | Mạch nạp ST-Link V2 | Xung nhịp debug SWD |
| **PA15** | `AIN2` | GPIO Output PP | Ra | Chân AIN2 trên TB6612FNG | Điều khiển chiều quay Động cơ Trái |
| **PB3** | `AIN1` | GPIO Output PP | Ra | Chân AIN1 trên TB6612FNG | Điều khiển chiều quay Động cơ Trái |
| **PB4** | `STBY` | GPIO Output PP | Ra | Chân STBY trên TB6612FNG | Kích hoạt IC (Mức 1 = Bật, Mức 0 = Chờ tiết kiệm điện) |
| **PB5** | `BIN1` | GPIO Output PP | Ra | Chân BIN1 trên TB6612FNG | Điều khiển chiều quay Động cơ Phải |
| **PB6** | `BIN2` | GPIO Output PP | Ra | Chân BIN2 trên TB6612FNG | Điều khiển chiều quay Động cơ Phải |
| **PB7** | `PWMA` | TIM4_CH2 (PWM) | Ra | Chân PWMA trên TB6612FNG | Điều chỉnh tốc độ Động cơ Trái (Xung 20 kHz) |
| **PB8** | `PWMB` | TIM4_CH3 (PWM) | Ra | Chân PWMB trên TB6612FNG | Điều chỉnh tốc độ Động cơ Phải (Xung 20 kHz) |

---

## 3. Lưu ý Đặc biệt: Giải phóng Chân JTAG (AFIO Remap)

Trên vi điều khiển STM32F103, các chân **PA15 (JTDI)**, **PB3 (JTDO)** và **PB4 (JNTRST)** mặc định sau khi reset thuộc về giao diện gỡ lỗi JTAG.

Để sử dụng các chân này làm GPIO thông thường điều khiển TB6612 mà **không làm mất tính năng nạp/debug ST-Link (SWD)**, mã nguồn tại `bsp_motor.c` thực hiện:
```c
__HAL_RCC_AFIO_CLK_ENABLE();
__HAL_AFIO_REMAP_SWJ_NOJTAG(); // Tắt JTAG, giữ lại cổng SWD trên PA13/PA14
```
> [!IMPORTANT]
> Tuyệt đối không gọi lệnh tắt toàn bộ SWJ (`SWJ_DISABLE`), nếu không bạn sẽ bị mất kết nối nạp qua ST-Link và phải nạp bằng chế độ BOOT0 / System Memory Serial.

---

## 4. Bảng Chân lý Điều khiển TB6612FNG

IC TB6612FNG điều khiển động cơ DC theo logic sau:

| IN1 | IN2 | PWM | STBY | Chế độ Động cơ (Operating Mode) |
| :---: | :---: | :---: | :---: | :--- |
| **1** | **0** | Duty | 1 | **Quay thuận (Forward)**: Tốc độ tỉ lệ với duty |
| **0** | **1** | Duty | 1 | **Quay nghịch (Backward)**: Tốc độ tỉ lệ với duty |
| **1** | **1** | X | 1 | **Phanh chủ động (Short Brake)**: Ngắn mạch cuộn dây, dừng nhanh |
| **0** | **0** | 0 | 1 | **Dừng trôi tự do (Coast / Stop)**: Thả nổi ngõ ra |
| X | X | X | **0** | **Chế độ chờ (Standby)**: Ngắt toàn bộ công suất, tiết kiệm điện |

> [!NOTE]
> **Đảo chiều Động cơ Trái bằng phần mềm**: Do đặc thù bố trí khung gầm đối xứng, Động cơ Trái được kích hoạt cờ `cfg_left.inverted = true` trong [`bsp_motor.c`](file:///d:/Workspace/Projects/AGV/agv_stm32/BSP/Src/bsp_motor.c) để đồng bộ chiều quay TIẾN chuẩn xác với Động cơ Phải. Chi tiết xem tại [`docs/motor_encoder_direction_guide.md`](motor_encoder_direction_guide.md).
>
> **Ánh xạ Duty Cycle phần cứng**: Xung PWM được ánh xạ từ dải chuẩn hóa `[0, 1000]` sang dải thanh ghi Timer 4 `[0, 3599]` ($\text{CCR} = \text{duty} \times 3599 / 1000$) để đảm bảo duty cycle và điện áp động cơ đạt chuẩn 100% công suất thực tế.

---

## 5. Cấu hình Cảm biến Encoder (Quadrature 4X)

- **Cấu hình Timer**: TIM2 và TIM3 được CubeMX cấu hình ở chế độ `TIM_ENCODERMODE_TI12` (chế độ đếm cả 2 sườn lên và xuống của cả 2 kênh A và B $\rightarrow$ nhân 4 độ phân giải).
- **Bộ lọc số chống nhiễu (Digital Filter = 6)**: Tích hợp bộ lọc 6 chu kỳ xung nhịp nội giúp loại trừ hoàn toàn các xung nhiễu gai cao tần từ chổi than động cơ DC.
- **Bù tràn số tự động**: Thanh ghi counter phần cứng là 16-bit (`0` đến `65535`). Thuật toán trong `Services/Src/encoder.c` ép kiểu toán học bù hai `(int16_t)(current_raw - last_raw)` để tự động xử lý tràn số mượt mà khi qua mốc `0` và `65535` mà không cần xử lý ngắt tràn.

---

## 6. Cấu hình Cảm biến 5 Mắt IR (ADC1 Scan Mode + DMA)

- **Các kênh ADC**: 5 kênh tương ứng với 5 mắt cảm biến quang học hồng ngoại từ phải sang trái:
  - Mắt 1 (Far Right - R2): PA2 (ADC1 Channel 2, DMA Index 0)
  - Mắt 2 (Right - R1): PA3 (ADC1 Channel 3, DMA Index 1)
  - Mắt 3 (Center - MID): PA4 (ADC1 Channel 4, DMA Index 2)
  - Mắt 4 (Left - L1): PA5 (ADC1 Channel 5, DMA Index 3)
  - Mắt 5 (Far Left - L2): PB0 (ADC1 Channel 8, DMA Index 4)
- **Cấu hình ADC1**: Scan Conversion Mode kích hoạt, Continuous Conversion kích hoạt.
- **DMA1 Channel 1**: Chế độ Circular Buffer ghi trực tiếp dữ liệu 12-bit (`0` - `4095`) vào mảng tĩnh nội bộ `s_ir_raw[5]` tại `bsp_ir.c`.
- **Hiệu chuẩn ADC nội**: `HAL_ADCEx_Calibration_Start(&hadc1)` được gọi tự động khi khởi tạo `BSP_IR_Init()` để khử sai số offset analog của bộ chuyển đổi ADC STM32F103.
- **Tần số đọc**: CPU không tốn thời gian chờ chuyển đổi; hàm `BSP_IR_GetRaw()` lấy dữ liệu tức thời $O(1)$ chỉ trong vài nano-giây.


