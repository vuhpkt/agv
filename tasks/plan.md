# Kế Hoạch Triển Khai: Điều Hướng Ngã Rẽ Sau Vạch Ngang & Bám Line 3 Mắt Phải

Tài liệu đặc tả cơ sở: [`docs/specs/SPEC-crossline-exit-turn.md`](file:///d:/Workspace/Projects/AGV/docs/specs/SPEC-crossline-exit-turn.md)

---

## 1. Tổng Quan Kiến Trúc (Architecture Overview)

Hiện tại, khi AGV phát hiện vạch ngang ngã rẽ (`s_tracker.is_crossline == true`) tại bước có lệnh `AGV_ACTION_TURN_RIGHT`, xe lập tức chuyển sang chế độ quay. Do cụm mắt cảm biến nhô ra phía trước đầu xe 10–15cm so với trục bánh, xe sẽ bẻ lái quá sớm khi bánh xe chưa vào tới tâm giao lộ, dẫn đến việc cắt cua gắt hoặc trôi lệch khỏi nhánh rẽ.

Kế hoạch này triển khai máy trạng thái điều hướng 2 giai đoạn:
1. **Giai đoạn 1 (Lướt qua vạch ngang - `LINE_STATE_CROSSING_FOR_TURN_RIGHT`):**
   - Xe tiếp tục chạy thẳng xuyên qua vạch ngang với tốc độ $v_L = v_R = v_{turn}$.
   - Theo dõi cạnh xuống (`falling edge`): khi cảm biến vừa trượt khỏi vạch ngang (`is_crossline` chuyển từ `true` sang `false`), bánh xe đã tiến vào đúng tâm ngã rẽ.
2. **Giai đoạn 2 (Dò 3 mắt phải qua Line PID - `LINE_STATE_TURNING_RIGHT`):**
   - Lập tức kích hoạt chế độ bám line 3 mắt phải `[IR0, IR1, IR2]` (triệt tiêu 2 mắt trái `norm[3]=0, norm[4]=0`).
   - Line PID tự động điều khiển vi sai 2 bánh, phanh bánh phải và tăng tốc bánh trái để xe tự vuốt cong mượt mà vào nhánh rẽ mới.
   - Bổ sung bộ đếm timeout an toàn 3.0 giây (`TURN_MAX_TIMEOUT_MS = 3000ms`) để ngắt khẩn cấp nếu bị mất vạch.

---

## 2. Phân Tích Đồ Thị Phụ Thuộc (Dependency Graph)

```
[SPEC-crossline-exit-turn.md]
              │
              ▼
    Task 1: Định nghĩa FSM & Hằng số
    (app_line_follow.h, app_line_follow.c)
              │
              ▼
    Task 2: Logic Lướt Vạch Ngang & Cạnh Xuống
    (app_line_follow.c)
              │
              ▼
    Task 3: Timeout An Toàn & PID 3 Mắt Phải
    (app_line_follow.c)
              │
              ▼
    Task 4: Unit Test Tự Động FSM & Edge Cases
    (tests/test_line_follow_edge_cases.py)
              │
              ▼
    Task 5: Biên Dịch Sạch Sẽ & Nghiệm Thu
    (cmake --build build/Debug)
```

---

## 3. Danh Sách Nhiệm Vụ Triển Khai (Task Breakdown)

### Task 1: Định Nghĩa Trạng Thái FSM & Hằng Số Điều Khiển
- **Files:** `agv_stm32/Application/Inc/app_line_follow.h`, `agv_stm32/Application/Src/app_line_follow.c`
- Bổ sung `LINE_STATE_CROSSING_FOR_TURN_RIGHT` vào `line_state_t`.
- Thêm `#define TURN_MAX_TIMEOUT_MS 3000`.
- Chuyển hướng khi phát hiện vạch ngang có lệnh `TURN_RIGHT` sang `LINE_STATE_CROSSING_FOR_TURN_RIGHT`.

### Task 2: Triển Khai Logic Lướt Qua Vạch Ngang & Cạnh Xuống (Falling Edge)
- **File:** `agv_stm32/Application/Src/app_line_follow.c`
- Thêm khối xử lý `LINE_STATE_CROSSING_FOR_TURN_RIGHT`.
- Duy trì chạy thẳng $v_L = v_R = v_{turn}$.
- Bắt cạnh xuống `!s_tracker.is_crossline` $\rightarrow$ chuyển `LINE_STATE_TURNING_RIGHT`, cập nhật `s_turn_start_tick`, tăng `s_route_step++`.

### Task 3: Tích Hợp Timeout 3.0s & Hoàn Thiện Line PID 3 Mắt Phải
- **File:** `agv_stm32/Application/Src/app_line_follow.c`
- Bổ sung kiểm tra `now - s_turn_start_tick >= TURN_MAX_TIMEOUT_MS` $\rightarrow$ dừng xe, gửi `AGV_MISSION_ERROR_LOST`.
- Bảo đảm triệt tiêu 2 mắt trái, tính sai số trọng số hình học $[1000, 385, 0]$ và điều khiển vi sai qua Line PID.

### Task 4: Xây Dựng Bộ Unit Test FSM & Cạnh Xuống
- **File:** `agv_stm32/tests/test_line_follow_edge_cases.py`
- Viết test kiểm tra trạng thái mới, hằng số timeout 3000ms và cấu trúc chuyển trạng thái cạnh xuống.
- Chạy toàn bộ test suite `py -3 -m unittest discover tests`.

### Task 5: Biên Dịch Firmware & Nghiệm Thu
- Lệnh: `cmake --build build/Debug`.
- Xác minh tài nguyên RAM/Flash, zero-malloc, không cảnh báo biên dịch.
