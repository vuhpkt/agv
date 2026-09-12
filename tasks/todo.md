# Danh Sách Nhiệm Vụ: Điều Hướng Ngã Rẽ Sau Vạch Ngang & Dò 3 Mắt Phải

Tài liệu đặc tả: [`docs/specs/SPEC-crossline-exit-turn.md`](file:///d:/Workspace/Projects/AGV/docs/specs/SPEC-crossline-exit-turn.md)
Kế hoạch tổng quan: [`tasks/plan.md`](file:///d:/Workspace/Projects/AGV/tasks/plan.md)

---

## Task 1: Định Nghĩa Trạng Thái FSM & Hằng Số Điều Khiển
**Description:** Bổ sung trạng thái `LINE_STATE_CROSSING_FOR_TURN_RIGHT` vào enum `line_state_t` và định nghĩa hằng số timeout an toàn `TURN_MAX_TIMEOUT_MS = 3000ms`. Chuyển hướng khi gặp vạch ngang có lệnh rẽ phải sang trạng thái lướt qua vạch ngang.

**Acceptance criteria:**
- [x] `app_line_follow.h` khai báo giá trị `LINE_STATE_CROSSING_FOR_TURN_RIGHT` trong `line_state_t`.
- [x] `app_line_follow.c` định nghĩa `#define TURN_MAX_TIMEOUT_MS 3000`.
- [x] Nhánh phát hiện vạch ngang khi `action == AGV_ACTION_TURN_RIGHT` chuyển sang `LINE_STATE_CROSSING_FOR_TURN_RIGHT` thay vì chuyển ngay sang trạng thái quay.

**Verification:**
- [x] `py -3 -m unittest discover tests` chạy không lỗi biên dịch cú pháp.

**Files touched:**
- `agv_stm32/Application/Inc/app_line_follow.h`
- `agv_stm32/Application/Src/app_line_follow.c`

---

## Task 2: Triển Khai Logic Lướt Qua Vạch Ngang & Cạnh Xuống (Falling Edge)
**Description:** Xây dựng khối xử lý cho `LINE_STATE_CROSSING_FOR_TURN_RIGHT` trong vòng lặp 100Hz. Xe duy trì đi thẳng với tốc độ $v_{turn}$ cho đến khi cụm cảm biến trượt khỏi vạch ngang (`is_crossline == false`), tại chu kỳ đó lập tức kích hoạt `LINE_STATE_TURNING_RIGHT`.

**Acceptance criteria:**
- [x] Khi ở `LINE_STATE_CROSSING_FOR_TURN_RIGHT`, xe đo cảm biến, cập nhật `is_crossline`, duy trì tốc độ 2 bánh $v_L = v_R = v_{turn}$.
- [x] Ngay chu kỳ đầu tiên `!s_tracker.is_crossline`, xe chuyển sang `LINE_STATE_TURNING_RIGHT`, lưu `s_turn_start_tick = now`, reset tracker PID.

**Verification:**
- [x] Test cấu trúc và logic chuyển cạnh xuống trong `tests/test_line_follow_edge_cases.py`.

**Files touched:**
- `agv_stm32/Application/Src/app_line_follow.c`

---

## Task 3: Tích Hợp Timeout 3.0s & Hoàn Thiện Line PID 3 Mắt Phải
**Description:** Bổ sung cơ chế bảo vệ an toàn ngắt motor sau 3.0 giây nếu không bắt được vạch mới khi đang rẽ, đồng thời hoàn thiện bộ tính sai số 3 mắt phải và Line PID ôm cua.

**Acceptance criteria:**
- [x] Trong `LINE_STATE_TURNING_RIGHT`, nếu `(now - s_turn_start_tick) >= TURN_MAX_TIMEOUT_MS` (3000ms), xe lập tức dừng khẩn cấp qua `App_LineFollow_Stop()` và gửi trạng thái `AGV_MISSION_ERROR_LOST`.
- [x] 2 mắt trái `norm[3], norm[4]` bị triệt tiêu bằng 0.
- [x] Sai số $e_{turn}$ tính theo trọng số hình học $[1000, 385, 0]$.
- [x] Điều khiển ôm cua bằng Line PID vi sai mượt mà.

**Verification:**
- [x] Chạy unit test kiểm tra công thức sai số và timeout.

**Files touched:**
- `agv_stm32/Application/Src/app_line_follow.c`

---

## Task 4: Xây Dựng Bộ Unit Test FSM & Edge Cases
**Description:** Viết các test case kiểm tra toàn diện trạng thái FSM mới, phát hiện cạnh xuống, timeout 3000ms và bộ trọng số cảm biến trong `test_line_follow_edge_cases.py`.

**Acceptance criteria:**
- [x] Test `test_crossing_for_turn_right_state_exists` đạt PASS.
- [x] Test `test_turn_max_timeout_defined` đạt PASS.
- [x] Test `test_falling_edge_turn_right_logic` đạt PASS.
- [x] Test `test_turn_right_timeout_protection` đạt PASS.
- [x] Toàn bộ test suite `tests/` đạt 100% (120/120 tests PASS).

**Verification:**
- [x] `py -3 -m unittest discover tests` đạt 100% PASS.

**Files touched:**
- `agv_stm32/tests/test_line_follow_edge_cases.py`

---

## Task 5: Biên Dịch Firmware & Nghiệm Thu Tổng Thể
**Description:** Biên dịch file thực thi `agv_stm32.elf`, kiểm tra tài nguyên bộ nhớ Flash và RAM, sẵn sàng nạp xuống kit STM32.

**Acceptance criteria:**
- [x] Lệnh `cmake --build build/Debug` hoàn tất thành công (0 errors, 0 warnings).
- [x] Bộ nhớ RAM = 26.60% (5448 B / 20 KB), Flash = 81.80% (53608 B / 64 KB).
- [x] Cập nhật walkthrough artifact và báo cáo cho người dùng.

**Verification:**
- [x] `cmake --build build/Debug` trả về returncode 0.
