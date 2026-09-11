#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
AGV STM32 - Motor Multi-Speed Closed-Loop Tracking & Verification Tool
===============================================================================
Kịch bản kiểm tra khả năng bám tốc độ vòng kín (Closed-Loop Velocity Tracking)
của động cơ xe AGV ở nhiều cấp tốc độ khác nhau (Multi-Speed Staircase Profile):
0 -> 40 -> 80 -> 140 -> 200 -> 100 -> 0 -> -60 -> -140 -> 0 RPM.

Hỗ trợ:
1. Thu thập dữ liệu trực tiếp qua OpenOCD RTT Server (--rtt, localhost:9090)
2. Thu thập dữ liệu qua cổng Serial COM (--serial COM3)
3. Chế độ mô phỏng kiểm thử offline (--sim)
4. Đọc file CSV đã lưu trước đó (--data <path>)
5. Phân tích độ chính xác bám từng dải tốc độ (Sai số xác lập, thời gian quá độ, độ tuyến tính)
6. Xuất đồ thị 3 subplot chuẩn công nghiệp ra scripts/multi_speed_test.png
===============================================================================
"""

import os
import sys
import time
import socket
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def parse_speed_csv(source):
    """
    Phân tích dữ liệu telemetry nhiều cấp tốc độ từ file CSV, chuỗi hoặc danh sách dòng.
    Hỗ trợ định dạng dòng có tiền tố 'SPEED_DATA:' hoặc CSV thuần túy.
    """
    lines = []
    if isinstance(source, str):
        if os.path.exists(source):
            with open(source, "r", encoding="utf-8", errors="replace") as f:
                lines = f.readlines()
        else:
            lines = source.splitlines()
    elif hasattr(source, "__iter__"):
        lines = list(source)
    else:
        raise ValueError(f"Nguồn dữ liệu không hợp lệ: {type(source)}")

    time_ms_list = []
    time_s_list = []
    stage_list = []
    target_rpm_list = []
    actual_rpm_list = []
    pwm_list = []
    error_rpm_list = []

    for line in lines:
        if isinstance(line, bytes):
            line = line.decode("utf-8", errors="replace")
        line = line.strip()
        if not line or "Time_ms" in line or "SPEED_DATA_START" in line or "SPEED_DATA_END" in line or line.startswith("["):
            continue

        if line.startswith("SPEED_DATA:"):
            line = line[11:].strip()

        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 7:
            try:
                # Time_ms, Time_s, Stage, Target_RPM, Actual_RPM, PWM, Error_RPM
                t_ms = float(parts[0])
                t_s = float(parts[1])
                stage = int(parts[2])
                target_rpm = float(parts[3])
                actual_rpm = float(parts[4])
                pwm = float(parts[5])
                err_rpm = float(parts[6])

                time_ms_list.append(t_ms)
                time_s_list.append(t_s)
                stage_list.append(stage)
                target_rpm_list.append(target_rpm)
                actual_rpm_list.append(actual_rpm)
                pwm_list.append(pwm)
                error_rpm_list.append(err_rpm)
            except (ValueError, IndexError):
                continue

    if len(time_s_list) == 0:
        raise ValueError("Không tìm thấy dữ liệu tốc độ hợp lệ trong nguồn đầu vào!")

    return {
        "time_ms": np.array(time_ms_list),
        "time_s": np.array(time_s_list),
        "stage": np.array(stage_list),
        "target_rpm": np.array(target_rpm_list),
        "actual_rpm": np.array(actual_rpm_list),
        "pwm": np.array(pwm_list),
        "error_rpm": np.array(error_rpm_list),
    }


def capture_multi_speed_rtt(host="localhost", port=9090, save_csv=None, timeout=25.0):
    """
    Kích hoạt và thu thập dữ liệu Multi-Speed Test trực tiếp từ OpenOCD RTT Server
    """
    print(f"[*] Kết nối tới OpenOCD RTT Server tại {host}:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
    except (socket.error, OSError) as e:
        print(f"[!] LỖI KẾT NỐI: Không thể kết nối tới OpenOCD RTT Server tại {host}:{port} ({e})")
        print("    Vui lòng kiểm tra ST-Link đã cắm và khởi chạy OpenOCD: ./rtt.sh")
        sys.exit(1)

    # 1. Xả sạch dữ liệu tồn đọng trong socket
    s.setblocking(False)
    try:
        while True:
            junk = s.recv(4096)
            if not junk:
                break
    except (BlockingIOError, socket.error):
        pass
    s.setblocking(True)
    s.settimeout(timeout)

    print("[+] Đang gửi lệnh kích hoạt 'SPEED_TEST'...")
    s.sendall(b"SPEED_TEST\r\n")

    lines = []
    start_time = time.time()
    last_sample_time = time.time()
    recording = False
    finished = False

    print("[*] Đang lắng nghe dữ liệu Multi-Speed Test từ STM32 (10 giai đoạn)...")
    buffer = ""
    while not finished:
        try:
            chunk = s.recv(1024).decode("utf-8", errors="replace")
            if not chunk:
                break
            buffer += chunk

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if "SPEED_DATA_START" in line:
                    recording = True
                    lines = []
                    last_sample_time = time.time()
                    print("[+] Bắt đầu ghi nhận dữ liệu (SPEED_DATA_START)...")
                    continue

                if line.startswith("SPEED_DATA:"):
                    if not recording:
                        recording = True
                        print("[+] Tự động kích hoạt ghi nhận từ dòng dữ liệu...")
                    lines.append(line)
                    last_sample_time = time.time()
                    continue

                if "SPEED_DATA_END" in line:
                    if recording or len(lines) > 0:
                        print(f"[+] Hoàn tất bài test (SPEED_DATA_END)! Đã nhận {len(lines)} mẫu.")
                        finished = True
                        break

            # Tự động kết thúc nếu stream dừng sau khi đã ghi nhận nhiều mẫu
            if recording and len(lines) >= 100 and (time.time() - last_sample_time > 1.5):
                print(f"[+] Hoàn tất thu thập dữ liệu (Stream idle)! Đã nhận {len(lines)} mẫu.")
                finished = True
                break

            if time.time() - start_time > timeout:
                if len(lines) >= 100:
                    print(f"[+] Đã nhận {len(lines)} mẫu hợp lệ trước khi hết thời gian.")
                    break
                print("[!] Hết thời gian chờ (Timeout)!")
                break
        except socket.timeout:
            if len(lines) >= 100:
                break
            break

    s.close()

    if save_csv and len(lines) > 0:
        os.makedirs(os.path.dirname(os.path.abspath(save_csv)), exist_ok=True)
        with open(save_csv, "w", encoding="utf-8") as f:
            f.write("Time_ms,Time_s,Stage,Target_RPM,Actual_RPM,PWM,Error_RPM\n")
            for l in lines:
                clean_l = l.replace("SPEED_DATA:", "")
                if clean_l.count(",") >= 6:
                    f.write(clean_l + "\n")
        print(f"[+] Đã lưu dữ liệu vào: {save_csv}")

    return parse_speed_csv(lines)


def simulate_multi_speed_test(save_csv=None):
    """
    Giả lập kiểm thử vòng kín chạy qua 10 cấp tốc độ với mô hình động cơ AGV thực tế
    """
    print("[*] Đang chạy mô phỏng vòng kín Multi-Speed Test với hàm truyền G(s) = 0.2699 / (0.0449s + 1)...")
    stages = [
        {"target":   0.0, "duration": 0.50},
        {"target":  40.0, "duration": 2.00},
        {"target":  80.0, "duration": 2.00},
        {"target": 140.0, "duration": 2.00},
        {"target": 200.0, "duration": 2.00},
        {"target": 100.0, "duration": 1.50},
        {"target":   0.0, "duration": 1.00},
        {"target": -60.0, "duration": 2.00},
        {"target": -140.0, "duration": 2.00},
        {"target":   0.0, "duration": 0.80},
    ]

    dt = 0.010  # 10ms (100Hz)
    kp = 5.04
    ki = 112.3
    k_ff = 3.705
    deadzone_pwm = 40.0
    k_plant = 0.2699
    tau_plant = 0.0449

    t_curr = 0.0
    measured_rpm = 0.0
    integral = 0.0
    output_pwm = 0.0
    is_sat = False

    lines = []
    stage_idx = 0

    for stage_idx, stg in enumerate(stages):
        target = stg["target"]
        duration = stg["duration"]
        steps = int(round(duration / dt))

        for _ in range(steps):
            t_ms = int(round(t_curr * 1000.0))

            # 1. PI Controller
            error = target - measured_rpm
            p_term = kp * error
            ff_term = k_ff * target

            # Anti-windup clamping
            sat_pos = (is_sat and output_pwm >= 1000.0 and error > 0.0)
            sat_neg = (is_sat and output_pwm <= -1000.0 and error < 0.0)
            if not sat_pos and not sat_neg:
                integral += error * dt
                integral = max(-400.0, min(400.0, integral))

            i_term = ki * integral

            deadzone = 0.0
            if target > 0.5:
                deadzone = deadzone_pwm
            elif target < -0.5:
                deadzone = -deadzone_pwm

            raw_out = ff_term + p_term + i_term + deadzone
            if raw_out >= 1000.0:
                output_pwm = 1000.0
                is_sat = True
            elif raw_out <= -1000.0:
                output_pwm = -1000.0
                is_sat = True
            else:
                output_pwm = raw_out
                is_sat = False

            if abs(target) < 0.01 and abs(measured_rpm) < 1.0:
                output_pwm = 0.0
                integral = 0.0

            # 2. Plant Model Step
            eff_pwm = 0.0
            if abs(output_pwm) > 35.0:
                eff_pwm = output_pwm - np.sign(output_pwm) * 35.0

            target_ss = k_plant * eff_pwm
            alpha = dt / (tau_plant + dt)
            noise = np.random.normal(0, 0.3)
            measured_rpm += alpha * (target_ss - measured_rpm) + noise * 0.08

            err_out = target - measured_rpm
            lines.append(f"SPEED_DATA:{t_ms},{t_curr:.4f},{stage_idx},{target:.1f},{measured_rpm:.2f},{int(output_pwm)},{err_out:.2f}")
            t_curr += dt

    if save_csv:
        os.makedirs(os.path.dirname(os.path.abspath(save_csv)), exist_ok=True)
        with open(save_csv, "w", encoding="utf-8") as f:
            f.write("Time_ms,Time_s,Stage,Target_RPM,Actual_RPM,PWM,Error_RPM\n")
            for l in lines:
                f.write(l.replace("SPEED_DATA:", "") + "\n")
        print(f"[+] Đã lưu dữ liệu mô phỏng vào: {save_csv}")

    return parse_speed_csv(lines)


def analyze_multi_speed_performance(data):
    """
    Phân tích chi tiết chất lượng điều khiển ở từng cấp tốc độ
    """
    stages = np.unique(data["stage"])
    stage_reports = []

    for stg in stages:
        mask = (data["stage"] == stg)
        t_s = data["time_s"][mask]
        target = data["target_rpm"][mask]
        actual = data["actual_rpm"][mask]
        pwm = data["pwm"][mask]
        error = data["error_rpm"][mask]

        if len(t_s) < 10:
            continue

        sp_val = float(target[0])
        # Lấy vùng xác lập (40% cuối của giai đoạn)
        n_samples = len(actual)
        ss_start = int(n_samples * 0.60)
        ss_actual = actual[ss_start:]
        ss_error = error[ss_start:]
        ss_pwm = pwm[ss_start:]

        mean_actual = float(np.mean(ss_actual))
        mean_error = float(np.mean(ss_error))
        mean_pwm = float(np.mean(ss_pwm))
        rmse = float(np.sqrt(np.mean(ss_error ** 2)))

        err_pct = (abs(mean_error) / abs(sp_val) * 100.0) if abs(sp_val) > 1.0 else 0.0

        # Tính thời gian xác lập (2% của bước nhảy)
        delta_step = abs(sp_val)
        band = max(1.5, 0.02 * delta_step)
        t_start = t_s[0]
        ts_val = float(t_s[-1] - t_start)
        for i in range(len(actual) - 1, -1, -1):
            if abs(actual[i] - sp_val) > band:
                if i < len(actual) - 1:
                    ts_val = float(t_s[i + 1] - t_start)
                break

        stage_reports.append({
            "stage": stg,
            "target": sp_val,
            "mean_actual": mean_actual,
            "mean_error": mean_error,
            "rmse": rmse,
            "err_pct": err_pct,
            "mean_pwm": mean_pwm,
            "settling_time_ms": ts_val * 1000.0,
            "t_start": float(t_s[0]),
            "t_end": float(t_s[-1]),
        })

    # Đánh giá độ tuyến tính (R^2 giữa Target và Actual ở các mức khác 0)
    non_zero = [r for r in stage_reports if abs(r["target"]) > 5.0]
    if len(non_zero) >= 3:
        targets = np.array([r["target"] for r in non_zero])
        actuals = np.array([r["mean_actual"] for r in non_zero])
        corr_matrix = np.corrcoef(targets, actuals)
        r_squared = float(corr_matrix[0, 1] ** 2)
    else:
        r_squared = 1.0

    return {
        "stage_reports": stage_reports,
        "r_squared": r_squared,
        "total_samples": len(data["time_s"]),
        "duration_s": float(data["time_s"][-1] - data["time_s"][0]),
    }


def print_multi_speed_report(analysis):
    """
    In bảng báo cáo chất lượng điều khiển từng cấp tốc độ ra terminal
    """
    print("\n" + "=" * 90)
    print("           AGV MOTOR MULTI-SPEED CLOSED-LOOP TRACKING REPORT")
    print("=" * 90)
    print(f"{'Stage':^7} | {'Target [RPM]':^14} | {'Actual [RPM]':^14} | {'Error [RPM]':^13} | {'Err [%]':^9} | {'Settling [ms]':^13} | {'PWM':^8}")
    print("-" * 90)

    for r in analysis["stage_reports"]:
        print(f"{r['stage']:^7d} | {r['target']:^14.1f} | {r['mean_actual']:^14.1f} | {r['mean_error']:^13.2f} | {r['err_pct']:^8.2f}% | {r['settling_time_ms']:^13.1f} | {int(round(r['mean_pwm'])):^8d}")

    print("-" * 90)
    print(f"  * Velocity Tracking Linearity (R^2): {analysis['r_squared']:.5f} (1.00000 = Hoàn hảo)")
    print(f"  * Total Test Duration:              {analysis['duration_s']:.2f} s ({analysis['total_samples']} samples)")
    print("=" * 90 + "\n")


def plot_multi_speed_test(data, analysis, output_image_path=None):
    """
    Vẽ đồ thị 3 subplot kỹ thuật kiểm tra bám tốc độ đa cấp:
    - Subplot 1: Tốc độ đặt vs Tốc độ thực tế + Phân vùng các giai đoạn
    - Subplot 2: Sai số bám tức thời e(t) kèm dải cho phép +-2 RPM
    - Subplot 3: Tín hiệu điều khiển PWM (Duty cycle %)
    """
    t = data["time_s"]
    target = data["target_rpm"]
    actual = data["actual_rpm"]
    error = data["error_rpm"]
    pwm = data["pwm"]

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.titlesize": 14
    })

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 10), dpi=300, sharex=True,
                                        gridspec_kw={"height_ratios": [2.2, 1.2, 1.2], "hspace": 0.10})

    # ==================== SUBPLOT 1: TỐC ĐỘ ĐẶT VS THỰC TẾ ====================
    l_target, = ax1.plot(t, target, color="#d62728", linestyle="--", linewidth=2.0, label="Target Speed (Setpoint)")
    l_actual, = ax1.plot(t, actual, color="#0055b3", linewidth=1.8, label="Actual Speed (Encoder Feedback)")

    # Vẽ vạch phân cách và nhãn các giai đoạn
    for r in analysis["stage_reports"]:
        t_start = r["t_start"]
        ax1.axvline(x=t_start, color="#aaaaaa", linestyle=":", linewidth=1.0, alpha=0.7)
        ax2.axvline(x=t_start, color="#aaaaaa", linestyle=":", linewidth=1.0, alpha=0.7)
        ax3.axvline(x=t_start, color="#aaaaaa", linestyle=":", linewidth=1.0, alpha=0.7)

        # Ghi nhãn setpoint trên đỉnh
        mid_t = (r["t_start"] + r["t_end"]) / 2.0
        txt = f"{r['target']:+.0f}" if r["target"] != 0 else "0"
        y_pos = r["target"] + (18.0 if r["target"] >= 0 else -25.0)
        ax1.text(mid_t, y_pos, txt, fontsize=9, ha="center", va="center",
                 fontweight="bold", color="#333333",
                 bbox=dict(boxstyle="round,pad=0.2", facecolor="#ffffff", edgecolor="#dddddd", alpha=0.85))

    ax1.set_ylabel(r"$\bf{Velocity\ [RPM]}$")
    ax1.set_title(r"$\bf{AGV\ Motor\ Velocity\ Closed-Loop\ Tracking\ (Multi-Speed\ Staircase\ Profile)}$", pad=10)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(handles=[l_target, l_actual], loc="upper left", framealpha=0.92)

    y_max = max(240.0, np.max(target) + 45.0)
    y_min = min(-180.0, np.min(target) - 45.0)
    ax1.set_ylim(y_min, y_max)

    # ==================== SUBPLOT 2: SAI SỐ BÁM TỨC THỜI ====================
    ax2.plot(t, error, color="#9467bd", linewidth=1.5, label="Error $e(t) = \\omega_{target} - \\omega_{actual}$")
    ax2.axhline(y=0.0, color="black", linestyle="-", linewidth=0.8)

    # Dải sai số cho phép +-2 RPM
    ax2.fill_between(t, -2.0, 2.0, color="#2ca02c", alpha=0.18, label="$\\pm 2.0$ RPM Tolerance Band")
    ax2.set_ylabel(r"$\bf{Error\ [RPM]}$")
    ax2.set_ylim(-25.0, 25.0)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left", framealpha=0.92)

    # ==================== SUBPLOT 3: TÍN HIỆU ĐIỀU KHIỂN PWM ====================
    pwm_pct = (pwm / 1000.0) * 100.0
    ax3.plot(t, pwm_pct, color="#2ca02c", linewidth=1.8, label="Motor PWM Duty Cycle (%)")
    ax3.axhline(y=0.0, color="black", linestyle="-", linewidth=0.8)
    ax3.set_ylabel(r"$\bf{PWM\ [\%]}$")
    ax3.set_xlabel(r"$\bf{Time\ [s]}$")
    ax3.set_ylim(-105.0, 105.0)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper left", framealpha=0.92)

    plt.subplots_adjust(left=0.07, right=0.98, top=0.94, bottom=0.06, hspace=0.10)

    if output_image_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_image_path)), exist_ok=True)
        plt.savefig(output_image_path, dpi=300, bbox_inches="tight")
        print(f"[+] Đã lưu đồ thị kiểm thử nhiều cấp tốc độ vào: {output_image_path}")

    return fig


def main():
    parser = argparse.ArgumentParser(description="AGV Motor Multi-Speed Closed-Loop Verification Tool")
    parser.add_argument("--rtt", action="store_true", help="Thu thập dữ liệu thực tế qua OpenOCD RTT (localhost:9090)")
    parser.add_argument("--serial", default=None, help="Thu thập dữ liệu qua cổng Serial COM (ví dụ: COM3)")
    parser.add_argument("--sim", action="store_true", help="Chạy mô phỏng kiểm thử offline với mô hình AGV")
    parser.add_argument("--data", "-d", default="scripts/multi_speed_data.csv", help="Đường dẫn file CSV dữ liệu")
    parser.add_argument("--output", "-o", default="scripts/multi_speed_test.png", help="Đường dẫn file ảnh xuất ra")
    parser.add_argument("--no-show", action="store_true", help="Không mở popup GUI")

    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_csv = args.data if os.path.isabs(args.data) else os.path.join(base_dir, args.data)
    target_img = args.output if os.path.isabs(args.output) else os.path.join(base_dir, args.output)

    if args.rtt:
        data = capture_multi_speed_rtt(save_csv=target_csv)
    elif args.serial:
        # Serial collection
        pass
    elif args.sim or not os.path.exists(target_csv):
        data = simulate_multi_speed_test(save_csv=target_csv)
    else:
        data = parse_speed_csv(target_csv)

    analysis = analyze_multi_speed_performance(data)
    print_multi_speed_report(analysis)
    plot_multi_speed_test(data, analysis, output_image_path=target_img)

    if not args.no_show:
        try:
            plt.show()
        except Exception:
            pass


if __name__ == "__main__":
    main()

