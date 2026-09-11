#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Công cụ Kiểm Tra & Chẩn Đoán Chiều Động Cơ & Encoder AGV qua RTT (Task 3)
Kết nối tới OpenOCD RTT Server (port 9090) để kiểm tra:
1. Chiều quay vật lý của động cơ (Motor A - Trái, Motor B - Phải).
2. Chiều đếm xung của Encoder (TIM2 cho Motor A, TIM3 cho Motor B).
3. Đưa ra hướng dẫn khắc phục cụ thể trên phần cứng, STM32CubeMX hoặc code.
"""

import sys
import socket
import time
import re
import argparse

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9090
DEFAULT_DURATION_MS = 1500
DEFAULT_PWM = 400

DIAG_ALL_OK = "ALL_OK"
DIAG_ENCODER_INVERTED = "ENCODER_INVERTED"
DIAG_MOTOR_INVERTED = "MOTOR_INVERTED"
DIAG_BOTH_INVERTED = "BOTH_INVERTED"
DIAG_NO_MOTION = "NO_MOTION"


def parse_rtt_step_response(rtt_output: str) -> dict:
    """
    Trích xuất kết quả bài step test từ chuỗi log RTT:
    Tìm dòng '[STEP_TEST] (TEST_PASS|TEST_FAIL) | Peak: ... | Steady: ... | Ticks: ...'
    """
    res = {
        "passed": False,
        "peak_rpm": 0.0,
        "steady_rpm": 0.0,
        "ticks": 0,
        "raw": rtt_output,
    }

    match = re.search(
        r"\[STEP_TEST\]\s+(TEST_PASS|TEST_FAIL)\s+\|\s+Peak:\s+([-\d.]+)\s+RPM\s+\|\s+Steady:\s+([-\d.]+)\s+RPM\s+\|\s+Ticks:\s+([-\d]+)",
        rtt_output,
    )
    if match:
        res["passed"] = (match.group(1) == "TEST_PASS")
        res["peak_rpm"] = float(match.group(2))
        res["steady_rpm"] = float(match.group(3))
        res["ticks"] = int(match.group(4))

    return res


def diagnose_result(pwm: int, wheel_spins_fwd: bool, ticks: int) -> dict:
    """
    Đánh giá tính đúng đắn của chiều quay động cơ và encoder:
    - pwm > 0: kỳ vọng bánh xe quay TIẾN và ticks tăng (+).
    - pwm < 0: kỳ vọng bánh xe quay LÙI và ticks giảm (-).
    """
    if abs(ticks) < 20:
        return {
            "status": DIAG_NO_MOTION,
            "wheel_ok": False,
            "encoder_ok": False,
            "recommendation": (
                "Không phát hiện xung encoder chuyển động (< 20 ticks).\n"
                "-> Kiểm tra nguồn cấp động cơ VM, chân STBY, hoặc dây kết nối Encoder."
            ),
        }

    # Bánh xe quay đúng hướng lệnh
    wheel_ok = (pwm > 0 and wheel_spins_fwd) or (pwm < 0 and not wheel_spins_fwd)

    # Encoder đếm đúng hướng thực tế bánh xe quay (Tiến -> Ticks +, Lùi -> Ticks -)
    encoder_ok = (wheel_spins_fwd and ticks > 0) or (not wheel_spins_fwd and ticks < 0)

    if wheel_ok and encoder_ok:
        status = DIAG_ALL_OK
        recom = "HOÀN HẢO: Động cơ quay đúng chiều lệnh và Encoder đếm đúng chiều (+Tiến / -Lùi). Không cần chỉnh sửa."
    elif wheel_ok and not encoder_ok:
        status = DIAG_ENCODER_INVERTED
        recom = (
            "CẢNH BÁO: Bánh xe quay đúng chiều tiến, nhưng Encoder đếm ngược!\n"
            "-> Khắc phục: Đảo Phase A/B vật lý, hoặc đổi Input Polarity trong STM32CubeMX (TIM2/TIM3 Encoder Mode),\n"
            "   hoặc đổi 'cfg.inverted = true' trong bsp_encoder.c."
        )
    elif not wheel_ok and encoder_ok:
        status = DIAG_MOTOR_INVERTED
        recom = (
            "CẢNH BÁO: Bánh xe quay ngược chiều lệnh PWM!\n"
            "-> Khắc phục: Đảo 2 dây động cơ (AO1/AO2 hoặc BO1/BO2), hoặc đổi chân AIN1/AIN2 (hoặc BIN1/BIN2) trong STM32CubeMX,\n"
            "   hoặc đổi 'cfg.inverted = true' trong bsp_motor.c."
        )
    else:
        status = DIAG_BOTH_INVERTED
        recom = (
            "CẢNH BÁO: Cả Động cơ và Encoder đều bị ngược!\n"
            "-> Khắc phục: Đảo 2 dây động cơ VÀ đổi Phase A/B encoder (hoặc cấu hình lại cả 2 trong STM32CubeMX)."
        )

    return {
        "status": status,
        "wheel_ok": wheel_ok,
        "encoder_ok": encoder_ok,
        "recommendation": recom,
    }


def execute_rtt_motor_test(motor: str, pwm: int, duration_ms: int = DEFAULT_DURATION_MS, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> dict:
    """
    Kết nối socket tới OpenOCD RTT server, gửi lệnh MOT, thu thập log và phân tích kết quả.
    """
    cmd = f"MOT {motor} {pwm} {duration_ms}\r\n"
    print(f"[>] Đang gửi lệnh qua RTT: {cmd.strip()} ...")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)

    try:
        s.connect((host, port))
    except Exception as e:
        print(f"[!] Không thể kết nối tới OpenOCD RTT Server tại {host}:{port}: {e}")
        print("    Vui lòng đảm bảo OpenOCD đang chạy bằng lệnh: ./rtt.sh")
        return {"passed": False, "ticks": 0, "peak_rpm": 0.0, "error": str(e)}

    received_data = ""
    timeout_s = (duration_ms / 1000.0) + 3.0
    start_time = time.time()

    try:
        s.sendall(cmd.encode("utf-8"))
        while time.time() - start_time < timeout_s:
            try:
                chunk = s.recv(1024).decode("utf-8", errors="ignore")
                if chunk:
                    received_data += chunk
                    if "DATA_END" in received_data and "[DIR_CHECK]" in received_data:
                        break
            except socket.timeout:
                break
    finally:
        s.close()

    return parse_rtt_step_response(received_data)


def interactive_single_test(motor: str, pwm: int, duration_ms: int = DEFAULT_DURATION_MS):
    motor_name = "Motor A (Trái)" if motor.upper() == "A" else "Motor B (Phải)"
    direction_label = "TIẾN (+PWM)" if pwm > 0 else "LÙI (-PWM)"

    print("\n" + "=" * 60)
    print(f"  BÀI TEST: {motor_name} - Chiều {direction_label} ({pwm} PWM, {duration_ms} ms)")
    print("=" * 60)
    print("(*) Lưu ý: Kê bánh xe hổng khỏi mặt bàn để xe không chạy tự do.")
    input("Nhấn ENTER để bắt đầu quay động cơ...")

    result = execute_rtt_motor_test(motor, pwm, duration_ms)

    if "error" in result:
        return

    ticks = result["ticks"]
    peak_rpm = result["peak_rpm"]

    print(f"\n[Kết quả RTT]: Ticks = {ticks} | Peak = {peak_rpm:.1f} RPM | Trạng thái = {'PASS' if result['passed'] else 'FAIL'}")

    while True:
        ans = input(f"\n[?] Quan sát thực tế: Bánh xe {motor_name} có quay TIẾN về phía đầu xe không? (y/n): ").strip().lower()
        if ans in ("y", "yes"):
            wheel_fwd = True
            break
        elif ans in ("n", "no"):
            wheel_fwd = False
            break
        print("Vui lòng nhập 'y' (có quay tiến) hoặc 'n' (quay lùi).")

    diag = diagnose_result(pwm, wheel_fwd, ticks)

    print("\n" + "-" * 60)
    print(f"  KẾT LUẬN CHẨN ĐOÁN: [{diag['status']}]")
    print(f"  - Chiều quay bánh xe : {'ĐÚNG' if diag['wheel_ok'] else 'NGƯỢC'}")
    print(f"  - Chiều đếm Encoder  : {'ĐÚNG' if diag['encoder_ok'] else 'NGƯỢC'} (Ticks: {ticks})")
    print("\n[Khuyến nghị]:")
    print(diag["recommendation"])
    print("-" * 60)


def main():
    parser = argparse.ArgumentParser(description="Chẩn đoán chiều quay Động cơ & Encoder AGV qua RTT")
    parser.add_argument("--test", choices=["A", "B"], help="Chỉ định động cơ test (A hoặc B)")
    parser.add_argument("--pwm", type=int, default=DEFAULT_PWM, help=f"Giá trị PWM (-1000 đến 1000, mặc định {DEFAULT_PWM})")
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION_MS, help=f"Thời gian test tính bằng ms (mặc định {DEFAULT_DURATION_MS})")
    args = parser.parse_args()

    if args.test:
        interactive_single_test(args.test, args.pwm, args.duration)
        return

    while True:
        print("\n=======================================================")
        print("  MENU KIỂM TRA CHIỀU ĐỘNG CƠ & ENCODER (TB6612 AGV)")
        print("=======================================================")
        print("  1. Test Motor A (Trái) TIẾN (+400 PWM, 1.5s)")
        print("  2. Test Motor A (Trái) LÙI  (-400 PWM, 1.5s)")
        print("  3. Test Motor B (Phải) TIẾN (+400 PWM, 1.5s)")
        print("  4. Test Motor B (Phải) LÙI  (-400 PWM, 1.5s)")
        print("  q. Thoát")
        print("=======================================================")
        choice = input("Chọn chức năng (1-4, q): ").strip().lower()

        if choice == "1":
            interactive_single_test("A", 400)
        elif choice == "2":
            interactive_single_test("A", -400)
        elif choice == "3":
            interactive_single_test("B", 400)
        elif choice == "4":
            interactive_single_test("B", -400)
        elif choice == "q":
            print("Tạm biệt!")
            break
        else:
            print("Lựa chọn không hợp lệ, vui lòng chọn lại.")


if __name__ == "__main__":
    main()
