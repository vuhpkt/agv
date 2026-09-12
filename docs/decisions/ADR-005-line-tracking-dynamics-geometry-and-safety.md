# ADR-005: Tối Ưu Động Học Bám Line, Cấu Hình Trọng Số Cảm Biến Theo Hình Học Thực Tế & Cơ Chế An Toàn Mất Vạch

## Trạng thái (Status)
**Đã chấp thuận (Accepted)**

## Ngày quyết định (Date)
2026-09-12

## Bối cảnh (Context)
Trong quá trình thử nghiệm thực tế hệ thống bám line (Line Tracking) trên sa bàn với các mức vận tốc khác nhau (từ 50 RPM đến 90 RPM), đội ngũ phát triển đã ghi nhận các vấn đề kỹ thuật sau:

1. **Phân bố vị trí hình học cảm biến thực tế không đều**:
   - Thanh 5 mắt cảm biến hồng ngoại trên xe có cấu trúc vật lý: mắt trung tâm (Mắt 3 - MID) cách 2 mắt kế cận (Mắt 2 và 4) khoảng cách **1.25 cm**, trong khi cách 2 mắt ngoài cùng (Mắt 1 và 5) khoảng cách **3.25 cm**.
   - Trọng số ban đầu được gán giả định tuyến tính đều: [-1000, -500, 0, +500, +1000]. Sự sai lệch hình học này khiến việc tính toán trọng tâm vạch e_line bị phi tuyến cục bộ, đánh giá sai độ lệch khi xe bắt đầu chớm rời tâm line.

2. **Lỗi chốt phân cực bẻ lái vi sai (Reversed Differential Polarity)**:
   - Một lỗi gán nhầm dấu vi sai (v_left = v_base - delta_v, v_right = v_base + delta_v khi delta_v > 0) từng khiến xe tạo vòng hồi tiếp dương: xe chớm lệch sang phải thì càng bẻ lái ngoặt sang phải và dính chặt vào mép ngoài cùng của vạch.

3. **Hiện tượng rung lắc dao động biên độ nhỏ (Hunting Oscillation) ở tốc độ thấp (50 RPM)**:
   - Trong module chẩn đoán diag_ir.c, bộ điều khiển Line PID ban đầu được gán K_i = 0.005f.
   - Trong bài toán bám line xe hai bánh vi sai, điểm đặt (Setpoint) luôn là tâm vạch (e = 0). Khâu tích phân K_i tích lũy sai số khi xe lệch nhẹ, gây ra hiện tượng trễ pha (phase lag). Đến khi xe đã quay về đúng tâm vạch, lượng tích phân tồn đọng vẫn ép xe tiếp tục bẻ lái sang phía đối diện, tạo ra dao động hình sin liên tục (hunting) làm mất ổn định cơ khí.

4. **Hiện tượng văng vạch ở khúc cua gắt với tốc độ cao (90 RPM)**:
   - Bộ điều khiển bị giới hạn độ bẻ lái vi sai ở out_max = 40.0 RPM.
   - Khi chạy ở tốc độ cơ sở v_base = 90 RPM, lượng chênh lệch bánh tối đa là delta_v = +-40 RPM. Bánh phía trong khúc cua chỉ giảm xuống 90 - 40 = 50 RPM (vẫn quay tiến rất nhanh!).
   - Bán kính quay vòng tối thiểu của xe khi đó bị kẹp ở:
     R_min = (W / 2) * (v_outer + v_inner) / (v_outer - v_inner) = (130 / 2) * (180 / 80) ≈ 146.25 mm.
   - Các khúc cua vuông 90° trên sa bàn thường có bán kính cong R <= 100 mm. Xe bị thiếu lái trầm trọng (understeer) và không thể bo hết góc cua, dẫn đến bị văng hoàn toàn ra khỏi vạch.

5. **Thiếu cơ chế ngắt an toàn khi mất vạch**:
   - Khi xe văng khỏi vạch trong các bài kiểm tra chẩn đoán LINE_TEST, firmware không có bộ đếm thời gian an toàn mà tiếp tục cho 2 bánh quay hết toàn bộ thời gian duration (10,000 ms), tiềm ẩn rủi ro va chạm cơ khí và hư hại thiết bị.

---

## Quyết định (Decision)

Dự án quyết định chuẩn hóa toàn diện động học bám line, hiệu chỉnh trọng số hình học và bổ sung cơ chế an toàn 2 giai đoạn:

### 1. Chuẩn Hóa Trọng Số Hình Học Cảm Biến Thực Tế
Căn cứ theo khoảng cách đo đạc vật lý (1.25 cm và 3.25 cm từ tâm):
- Trọng số tương đối của mắt phụ cận: (1.25 / 3.25) * 1000 ≈ 384.6 -> **385**.
- Thứ tự mảng trọng số chuẩn theo các kênh ADC [CH0(R2), CH1(R1), CH2(MID), CH3(L1), CH4(L2)]:
  W = [+1000, +385, 0, -385, -1000]
- Đảm bảo công thức sai số trọng tâm bậc 1 (Weighted Centroid) phản ánh đúng khoảng cách mm thực tế của xe so với tâm đường line.

### 2. Chuẩn Hóa Phân Cực Bẻ Lái Vi Sai Hồi Tiếp Âm
Áp dụng thống nhất trên toàn bộ firmware:
- v_left = v_base + delta_v
- v_right = v_base - delta_v
- Khi e > 0 (vạch nằm lệch bên phải xe): delta_v > 0 => v_left > v_right => Xe bẻ lái sang **PHẢI** để đưa tâm xe về vạch.
- Khi e < 0 (vạch nằm lệch bên trái xe): delta_v < 0 => v_left < v_right => Xe bẻ lái sang **TRÁI** để đưa tâm xe về vạch.

### 3. Cấu Trúc Bộ Điều Khiển Line PID & Triệt Tiêu Khâu Tích Phân
- **Loại bỏ hoàn toàn khâu tích phân**: Đặt K_i = 0.0f cho tất cả các chế độ bám line (pp_line_follow.c và diag_ir.c).
- **Tăng cường khâu vi sai & tỉ lệ**:
  - K_p = 0.15f: Phản ứng góc đánh lái nhạy bén khi chớm lệch tâm.
  - K_d = 0.035f: Tạo mô-men cản giảm chấn (damping), dập tắt dao động và triệt tiêu rung lắc hình sin.

### 4. Cơ Chế Tự Động Mở Rộng Giới Hạn Bẻ Lái Thích Ứng (out_max)
Thay vì cố định out_max ở một hằng số nhỏ, hệ thống tự động co giãn giới hạn đánh lái theo vận tốc cơ sở:
out_max = max(60.0f, v_base)
- Khi chạy ở tốc độ cao v_base = 90 RPM, out_max được mở rộng lên 90.0 RPM.
- Khi gặp góc cua vuông gắt, lượng vi sai cực đại đạt delta_v = 90 RPM:
  v_outer = 90 + 90 = 180 RPM, v_inner = 90 - 90 = 0 RPM.
- Bánh trong được **phanh dừng hẳn**, cho phép xe xoay tròn quanh tâm bánh trong với bán kính quay vòng:
  R_min = W / 2 ≈ 65 mm.
- Xe ôm trọn khúc cua gắt 90° mà không còn bị hiện tượng thiếu lái (understeer).

### 5. Cơ Chế An Toàn Mất Vạch 2 Giai Đoạn (Two-Phase Lost-Line Safety)
Định nghĩa thời gian ngắt an toàn thống nhất:
- LINE_LOST_TIMEOUT_MS = 2000 (app_line_follow.c)
- DIAG_IR_LOST_TIMEOUT_MS = 2000 (diag_ir.h)

- **Giai đoạn 1: Dò tìm chậm (Thời gian mất line < 2.0 s)**:
  - Khi tất cả 5 mắt cảm biến mất tín hiệu vạch (line_lost == true), xe tự động giảm tốc xuống 60% vận tốc cơ sở (v_search = 0.6 * v_base, tối thiểu 30 RPM).
  - Bẻ lái mạnh theo hướng lệch hợp lệ cuối cùng (last_valid_error) để quay đầu quét lại vạch tại khúc cua.
  - Nếu bắt lại được vạch line trong vòng 2 giây, bộ đếm thời gian s_lost_start_tick lập tức reset về 0 và xe tiếp tục bám line bình thường.
- **Giai đoạn 2: Dừng khẩn cấp (Thời gian mất line >= 2.0 s)**:
  - Tức thời ngắt điều khiển động cơ (App_MotorControl_Stop()), khóa cứng 2 bánh.
  - Ghi nhận trạng thái thất bại (DIAG_IR_STATE_FAILED hoặc AGV_MISSION_ERROR_LOST) và phát log cảnh báo khẩn cấp lên Gateway.

---

## Hệ quả & Đánh giá (Consequences)

### Tích Cực (Positive)
- **Triệt tiêu hiện tượng rung lắc**: Xe chuyển động thẳng êm ái, bám sát tâm vạch mà không có hiện tượng lắc lư hình sin.
- **Ôm cua chính xác ở dải tốc độ cao**: Thử nghiệm ở 90 RPM thành công qua các khúc cua gắt nhờ bánh trong có khả năng phanh đứng (0 RPM).
- **Bảo vệ an toàn phần cứng**: Xe tự động dừng hẳn sau đúng 2.0 giây nếu bị nhấc khỏi sàn hoặc văng ra khỏi đường chạy, loại bỏ hoàn toàn nguy cơ va chạm mất kiểm soát.
- **Tính toán chính xác theo hình học vật lý**: Trọng số cảm biến phản ánh trung thực kích thước mm của thanh cảm biến quang học.
- **Kiểm thử tự động bảo vệ**: Toàn bộ các ca kiểm thử hồi quy được tích hợp vào 	est_diag_ir.py và 	est_app_line_follow.py, duy trì 100% tỷ lệ pass.

### Hạn Chế / Lưu Ý (Trade-offs & Considerations)
- Ở tốc độ rất cao (v_base > 100 RPM), bánh ngoài khi bẻ cua gắt có thể yêu cầu tốc độ lên tới v_outer = 2 * v_base > 200 RPM. Cần đảm bảo điện áp nguồn pin cung cấp đủ công suất cực đại (xem thêm phân tích Headroom trong docs/control_theory.md).
