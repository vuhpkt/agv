# Hướng Dẫn Sử Dụng Bộ Công Cụ Python Và Giao Diện Điều Khiển

Tài liệu này hướng dẫn chi tiết cách cài đặt và sử dụng các công cụ Python trên máy tính để kết nối không dây qua Bluetooth Low Energy (BLE), thu thập dữ liệu đo xa, tự động nhận dạng động cơ và tinh chỉnh các bộ điều khiển PID cho xe tự hành AGV.

---

## 1. Cài Đặt Môi Trường

Các công cụ Python yêu cầu phiên bản Python 3.10 trở lên. Cài đặt toàn bộ các thư viện phụ thuộc bằng lệnh:

```bash
pip install -r requirements.txt
```

Các thư viện chính bao gồm:
- **`bleak`**: Giao tiếp Bluetooth Low Energy đa nền tảng (Windows, Linux, macOS).
- **`customtkinter`**: Giao diện đồ họa hiện đại hỗ trợ Dark Theme.
- **`matplotlib`**: Vẽ đồ thị đặc tuyến đáp ứng bước nhảy thời gian thực.
- **`numpy`**: Tính toán đại số tuyến tính và xử lý chuỗi số liệu đo xa.

---

## 2. Giao Diện Đồ Họa Desktop GUI Dashboard (`agv_ble_gui.py`)

Đây là công cụ đồ họa trực quan chính phục vụ vận hành, chẩn đoán và căn chỉnh tham số xe. Khởi chạy bằng lệnh:

```bash
py -3 agv_esp32/tools/agv_ble_gui.py
```

### 2.1. Thanh Trạng Thái Và Kết Nối BLE (Header Bar)
- **Quét và kết nối**: Ứng dụng tự động quét thiết bị có tên phát quảng bá `AGV_ESP32` và kết nối qua dịch vụ Nordic UART Service (NUS).
- **Trạng thái kết nối**: Hiển thị nhãn `CONNECTED (MTU: 256)` khi kết nối thành công.
- **Nút Dừng Khẩn Cấp (EMERGENCY STOP)**: Nút màu đỏ nổi bật ở góc trên cùng bên phải. Bấm nút này sẽ phát ngay lệnh `AGV_MSG_EMERGENCY_STOP` xuống xe để ngắt công suất và phanh cứng hai bánh xe trong mọi tình huống.
- **Nút PING**: Kiểm tra đường truyền UART giữa ESP32 và STM32 (LED PC13 trên STM32 sẽ nhấp nháy phản hồi).

---

### 2.2. Tab 1: Giám Sát Cảm Biến Và Điều Khiển Tay (Dashboard & Live Sensors)
Dùng để kiểm tra mảng cảm biến quang học và thử nghiệm chuyển động cơ bản của xe:
1. **Thanh đo 5 cảm biến IR (20Hz)**:
   - Hiển thị 5 thanh tiến trình trực quan đại diện cho 5 mắt cảm biến từ phải sang trái: `R2, R1, MID, L1, L2`.
   - Hiển thị đồng thời giá trị ADC thô ($0 \dots 4095$) và giá trị chuẩn hóa ($0 \dots 1000$).
   - Thước đo hiển thị độ lệch tâm vạch $e_{line}$ (tính theo milimét).
2. **Bộ điều khiển phím D-Pad**:
   - Cho phép điều khiển xe tiến, lùi, quay trái, quay phải và dừng xe bằng nút bấm hoặc thanh trượt điều chỉnh tốc độ từ $10$ đến $120\text{ RPM}$.
3. **Bộ đếm Encoder hai bánh**:
   - Hiển thị số xung đếm được thời gian thực từ hai bánh xe và nút đặt lại bộ đếm về 0.

---

### 2.3. Tab 2: Đáp Ứng Bước Nhảy Và Tự Động Dò Tham Số PI Động Cơ (Motor Step Response & Auto-Tuner)
Đây là công cụ quan trọng nhất để tinh chỉnh bộ điều khiển vận tốc bánh xe mà không cần tính toán thủ công:

#### Quy Trình Nhận Dạng Động Cơ Và Tự Động Tính PID:
1. **Kê hổng bánh xe** khỏi mặt sàn hoặc đặt xe trên bàn test.
2. Chọn động cơ cần kiểm tra: **Bánh Trái (Motor Left)** hoặc **Bánh Phải (Motor Right)**.
3. Cài đặt mức xung kích thích: Khuyên dùng mức **$500 \dots 800\text{ PWM}$** (tương ứng $50\% \dots 80\%$ công suất).
4. Cài đặt thời gian chạy: Mặc định **$1500\text{ ms}$** đến **$2500\text{ ms}$**.
5. Nhấn nút **RUN STEP TEST**:
   - Xe sẽ kích hoạt động cơ chạy ở mức PWM đã chọn.
   - Dữ liệu vận tốc thời gian thực được stream ngược lên qua BLE ở chu kỳ $10\text{ ms}$.
   - Đồ thị Matplotlib nhúng trên giao diện sẽ vẽ đặc tuyến quá độ $\Omega(t)$.
6. **Thuật toán Auto-Tuner tự động phân tích**:
   - Xác định vận tốc xác lập $\Omega_{ss}$ và tính hệ số khuếch đại tĩnh quy đổi về $100\%$ công suất ($K = \Omega_{ss} / U_{step}$).
   - Tìm hằng số thời gian quán tính $\tau = t_{63.2\%}$.
   - Tự động tính toán bộ hệ số tối ưu:
     - Hệ số bù tiến định: $K_{ff} = 1000 / \text{RPM}_{max}$.
     - Hệ số tỉ lệ: $K_p = \frac{\tau}{K \cdot \tau_{cl}}$ (với thời gian đáp ứng vòng kín mục tiêu $\tau_{cl} = 30\text{ ms}$).
     - Hệ số tích phân: $K_i = K_p / \tau$.
   - Đề xuất vận tốc bám line an toàn: $v_{safe} = (50\% \sim 65\%) \times \text{RPM}_{max}$, đảm bảo dự phòng từ $35\%$ đến $50\%$ công suất cho bánh ngoài khi vào cua gấp.
7. **Đồng bộ tham số xuống xe**:
   - Nhấn nút **Sync Tuned PI to STM32**.
   - Giao diện sẽ gửi gói tin `AGV_MSG_DIAG_PI_TUNE` xuống STM32 để áp dụng ngay bộ thông số mới mà không cần biên dịch lại firmware.
8. **Xuất dữ liệu**:
   - Nhấn nút **Export CSV** để lưu chuỗi số liệu đo phục vụ phân tích chuyên sâu trên MATLAB hoặc Excel nếu cần.

---

### 2.4. Tab 3: Hiệu Chuẩn Ngưỡng Cảm Biến Quang Học (Line Optical Calibration)
Dùng để nhận diện độ tương phản giữa sàn nhà và vạch dẫn hướng:
1. Đặt xe trên mặt sàn có vạch thực tế.
2. Nhấn nút **START CALIBRATION (10 SECONDS)**.
3. Trong vòng 10 giây, dùng tay lia mảng cảm biến qua lại ngang qua vạch đen và nền trắng của sàn nhà.
4. Thuật toán tự động ghi nhận giá trị nhỏ nhất (nền trắng) và lớn nhất (vạch đen) của từng mắt cảm biến.
5. Kiểm tra điều kiện dải động quang học: Mỗi mắt phải đạt $\Delta = Max - Min \ge 100$.
6. Nhấn nút **Save Calibration C Header**:
   - Ứng dụng sẽ tự động sinh và lưu trực tiếp file cấu hình [`agv_stm32/Application/Inc/app_line_calib_defaults.h`](file:///d:/Workspace/Projects/AGV/agv_stm32/Application/Inc/app_line_calib_defaults.h) với đúng định dạng macro `APP_LINE_CALIB_MIN_RAW` và `APP_LINE_CALIB_MAX_RAW`.

---

### 2.5. Tab 4: Lập Trình Lộ Trình Tự Hành (Route Mission Planner)
Dùng để thiết lập kịch bản di chuyển qua mạng lưới các ngã rẽ:
1. Đặt vận tốc bám vạch mong muốn (ví dụ $80\text{ RPM}$).
2. Thêm chuỗi các hành vi qua từng nút giao:
   - **Forward**: Bám line chạy thẳng xuyên qua ngã rẽ.
   - **Turn Left**: Rẽ trái tại ngã rẽ.
   - **Turn Right**: Lướt qua vạch ngang và kích hoạt rẽ phải 3 cảm biến.
   - **Stop**: Dừng xe tại trạm đích.
3. Nhấn **LOAD ROUTE** để nạp danh sách bước vào hàng đợi của STM32.
4. Nhấn **START MISSION** để bắt đầu chạy tự hành.
5. Nút **Demo Route**: Nạp nhanh chuỗi kịch bản mẫu `[Forward -> Turn Right -> Stop]` để chạy thử nghiệm ngã rẽ.

---

### 2.6. Khung Nhập Lệnh Dòng Lệnh Trực Tiếp (Console Tab)
Ở dưới cùng của ứng dụng có một khung dòng lệnh tương tác trực tiếp với firmware:
- `SET_SPEED <left_pwm> <right_pwm>`: Kích xung PWM vòng hở.
- `RPM <left_rpm> <right_rpm>`: Chạy điều khiển vận tốc kín bằng bộ PI.
- `PI_TUNE <kp> <ki> <kff>`: Cập nhật thủ công bộ hệ số PI động cơ.
- `LINE_START <rpm>`: Bật chế độ tự hành bám line liên tục.
- `LINE_STOP`: Dừng bám line và phanh dừng xe.
- `STOP`: Thả trôi dừng xe.

---

## 3. Công Cụ Dòng Lệnh Không Dây BLE CLI (`agv_ble_controller.py`)

Nếu muốn làm việc nhanh qua terminal hoặc viết script tự động hóa mà không cần mở giao diện đồ họa, sử dụng công cụ CLI:

```bash
py -3 agv_esp32/tools/agv_ble_controller.py
```

### Các Lệnh Hỗ Trợ Nhanh:
1. **Kiểm tra đáp ứng động cơ**:
   ```text
   step left 600 2000
   step right 600 2000
   ```
   Chạy động cơ với mức PWM 600 trong 2000ms và in kết quả đo vận tốc.
2. **Hiệu chuẩn cảm biến quang học**:
   ```text
   calibrate 10000
   ```
   Bật chế độ quét dải động quang học trong 10 giây và tự động xuất file header.
3. **Theo dõi thanh đo cảm biến IR trên terminal**:
   ```text
   monitor_ir 30000
   ```
   Vẽ thanh đo Unicode 5 mắt cảm biến trực tiếp trong cửa sổ dòng lệnh.
4. **Cập nhật hệ số PI từ xa**:
   ```text
   pi_tune 5.0 110.0 3.7
   ```
5. **Nạp và chạy lộ trình**:
   ```text
   route
   ```

---

## 4. Các Script Phân Tích Chạy Qua Cáp Nối Tiếp / RTT (`agv_stm32/scripts/`)

Khi làm việc tĩnh tại bàn kỹ thuật với cáp nạp ST-Link:
- **`test_motor_direction.py`**: Chẩn đoán chiều quay của từng động cơ và chiều đếm xung encoder, đối chiếu với ma trận đảo chiều trong firmware.
- **`plot_step_response.py`**: Đọc log dữ liệu đo bước nhảy từ file hoặc cổng RTT và xuất file ảnh đồ thị `step_response.png`.
- **`calibrate_line.py`**: Hiệu chuẩn ngưỡng quang học khi kết nối qua cổng nối tiếp USB-TTL.
