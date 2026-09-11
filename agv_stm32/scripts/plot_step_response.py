#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
AGV STM32 - Motor Step Response Identification & Plotting Tool
===============================================================================
Script thu thập và phân tích đáp ứng xung đơn vị (Unit Step Response 80% PWM)
cho động cơ DC TB6612 trên xe tự hành AGV STM32F103.

Hỗ trợ:
1. Đọc dữ liệu từ file CSV hoặc trực tiếp từ stream bộ nhớ
2. Kết nối trực tiếp qua OpenOCD RTT Server (localhost:9090)
3. Kết nối trực tiếp qua Serial COM Port (USB-UART / Virtual COM)
4. Tự động nhận dạng hệ thống chuẩn lý thuyết điều khiển tự động:
   - Vận tốc xác lập (Steady-state speed omega_ss)
   - Hệ số khuếch đại tĩnh (DC Gain K)
   - Hằng số thời gian (Time Constant tau - 63.2%) có nội suy tuyến tính
   - Thời gian tăng (Rise Time tr 10% -> 90%) có nội suy tuyến tính
   - Thời gian quá độ / xác lập (Settling Time ts - dải sai số 2% và 5%)
   - Độ vọt lố (Overshoot %)
   - Mô hình hàm truyền FOPTD: G(s) = K / (tau*s + 1)
   - Bộ thông số PI / PID theo Ziegler-Nichols
5. Xuất đồ thị chuẩn công nghiệp 2 subplot ra scripts/step_response.png
===============================================================================
"""

import os
import sys
import time
import argparse
import socket
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.ticker as ticker

# Đảm bảo mã hóa UTF-8 cho stdout trên Windows
if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def parse_csv_file(source):
    """
    Đọc và phân tích dữ liệu telemetry từ file CSV, chuỗi nội dung hoặc danh sách các dòng log.
    Hỗ trợ cả định dạng dòng có tiền tố 'DATA:' hoặc định dạng CSV thuần túy.
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

    time_s_list = []
    pwm_list = []
    pwm_pct_list = []
    delta_ticks_list = []
    rpm_list = []
    rads_list = []

    for line in lines:
        if isinstance(line, bytes):
            line = line.decode("utf-8", errors="replace")
        line = line.strip()
        if not line:
            continue

        # Bỏ qua các dòng tiêu đề, log trạng thái hoặc marker
        if "Time_ms" in line or "DATA_START" in line or "DATA_END" in line or line.startswith("["):
            continue

        # Loại bỏ tiền tố DATA: nếu có
        if line.startswith("DATA:"):
            line = line[5:].strip()

        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 6:
            try:
                # Cột: Time_ms, Time_s, PWM, PWM_Percent, DeltaTicks, Speed_RPM, Speed_RadS
                t_s = float(parts[1])
                pwm = float(parts[2])
                pwm_pct = float(parts[3])
                delta = int(float(parts[4]))
                rpm = float(parts[5])
                rads = float(parts[6]) if len(parts) > 6 else rpm * (2.0 * np.pi / 60.0)

                time_s_list.append(t_s)
                pwm_list.append(pwm)
                pwm_pct_list.append(pwm_pct)
                delta_ticks_list.append(delta)
                rpm_list.append(rpm)
                rads_list.append(rads)
            except (ValueError, IndexError):
                continue

    if len(time_s_list) == 0:
        raise ValueError("Không tìm thấy dữ liệu telemetry hợp lệ trong nguồn đầu vào!")

    return {
        "time_s": np.array(time_s_list),
        "pwm": np.array(pwm_list),
        "pwm_pct": np.array(pwm_pct_list),
        "delta": np.array(delta_ticks_list),
        "rpm": np.array(rpm_list),
        "rads": np.array(rads_list),
    }


def capture_from_rtt(host="localhost", port=9090, save_csv=None, timeout=10.0):
    """
    Thu thập dữ liệu trực tiếp từ OpenOCD RTT Server qua TCP Socket.
    Không tạo file tạm thời, phân tích trực tiếp trong bộ nhớ.
    """
    print(f"[*] Kết nối tới OpenOCD RTT Server tại {host}:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
    except (socket.error, OSError) as e:
        print(f"[!] LỖI KẾT NỐI: Không thể kết nối tới OpenOCD RTT Server tại {host}:{port} ({e})")
        print("    Vui lòng kiểm tra ST-Link đã cắm và khởi chạy OpenOCD RTT server trước:")
        print("    -> Chạy lệnh: ./rtt.sh (hoặc cmake --build build/Debug -t flash)")
        sys.exit(1)
    # Xả sạch toàn bộ dữ liệu tồn đọng trong socket (nếu có) trước khi gửi lệnh kích hoạt
    s.setblocking(False)
    drained_bytes = 0
    try:
        while True:
            junk = s.recv(4096)
            if not junk:
                break
            drained_bytes += len(junk)
    except (BlockingIOError, socket.error):
        pass
    s.setblocking(True)
    s.settimeout(timeout)
    if drained_bytes > 0:
        print(f"[*] Đã xả {drained_bytes} bytes dữ liệu đệm cũ tồn đọng từ OpenOCD RTT.")

    print("[+] Đang gửi lệnh kích hoạt 'STEP_TEST'...")
    s.sendall(b"STEP_TEST\r\n")

    lines = []
    start_time = time.time()
    last_sample_time = time.time()
    recording = False
    finished = False

    print("[*] Đang lắng nghe dữ liệu từ STM32...")
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

                if "DATA_START" in line:
                    recording = True
                    lines = []  # Đảm bảo reset sạch cho phiên test mới
                    last_sample_time = time.time()
                    print("[+] Bắt đầu ghi nhận mẫu xung (DATA_START)...")
                    continue

                # Tự động kích hoạt ghi nhận nếu bắt gặp dòng telemetry dữ liệu (phòng trường hợp mất gói DATA_START)
                if line.startswith("DATA:"):
                    if not recording:
                        recording = True
                        print("[+] Tự động kích hoạt ghi nhận từ dòng telemetry (DATA)...")
                    lines.append(line)
                    last_sample_time = time.time()
                    continue

                if "DATA_END" in line:
                    if recording or len(lines) > 0:
                        print(f"[+] Hoàn tất thu thập dữ liệu (DATA_END)! Đã nhận {len(lines)} mẫu.")
                        finished = True
                        break
                    else:
                        # Bỏ qua cờ DATA_END cũ còn sót lại từ các lần chạy trước
                        continue

                if recording and line and line.count(",") >= 5:
                    lines.append(line)
                    last_sample_time = time.time()

            # Tự động kết thúc nếu đã thu thập nhiều mẫu và không còn dòng dữ liệu mới trong 1.0 giây
            if recording and len(lines) >= 30 and (time.time() - last_sample_time > 1.0):
                print(f"[+] Hoàn tất thu thập dữ liệu (Stream idle)! Đã nhận {len(lines)} mẫu.")
                finished = True
                break

            if time.time() - start_time > timeout:
                if len(lines) >= 30:
                    print(f"[+] Đã nhận {len(lines)} mẫu hợp lệ trước khi hết thời gian.")
                    break
                print("[!] Hết thời gian chờ (Timeout)!")
                break
        except socket.timeout:
            if len(lines) >= 30:
                break
            break

    s.close()

    if save_csv and len(lines) > 0:
        with open(save_csv, "w", encoding="utf-8") as f:
            f.write("Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS\n")
            for l in lines:
                clean_l = l.replace("DATA:", "")
                if clean_l.count(",") >= 5:
                    f.write(clean_l + "\n")
        print(f"[+] Đã lưu dữ liệu vào: {save_csv}")

    return parse_csv_file(lines)


def capture_from_serial(port, baudrate=115200, save_csv=None, timeout=10.0):
    """
    Thu thập dữ liệu trực tiếp qua cổng USB Serial COM Port (dùng pyserial).
    Không tạo file tạm thời, phân tích trực tiếp trong bộ nhớ.
    """
    try:
        import serial
    except ImportError:
        print("[!] Thư viện 'pyserial' chưa được cài đặt. Hãy chạy: pip install pyserial")
        sys.exit(1)

    print(f"[*] Mở cổng Serial {port} (Baudrate: {baudrate})...")
    ser = serial.Serial(port, baudrate, timeout=1.0)
    time.sleep(0.5)  # Chờ DTR reset ổn định
    ser.reset_input_buffer()  # Xả sạch dữ liệu tồn đọng trong cổng COM

    print("[+] Đang gửi lệnh kích hoạt 'STEP_TEST'...")
    ser.write(b"STEP_TEST\r\n")

    lines = []
    start_time = time.time()
    last_sample_time = time.time()
    recording = False

    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            if recording and len(lines) >= 30 and (time.time() - last_sample_time > 1.0):
                print(f"[+] Hoàn tất thu thập dữ liệu (Stream idle)! Đã nhận {len(lines)} mẫu.")
                break
            if time.time() - start_time > timeout:
                if len(lines) >= 30:
                    break
                break
            continue

        if "DATA_START" in line:
            recording = True
            lines = []
            last_sample_time = time.time()
            print("[+] Bắt đầu ghi nhận mẫu xung (DATA_START)...")
            continue

        if line.startswith("DATA:"):
            if not recording:
                recording = True
                print("[+] Tự động kích hoạt ghi nhận từ dòng telemetry (DATA)...")
            lines.append(line)
            last_sample_time = time.time()
            continue

        if "DATA_END" in line:
            if recording or len(lines) > 0:
                print(f"[+] Hoàn tất thu thập dữ liệu (DATA_END)! Đã nhận {len(lines)} mẫu.")
                break
            else:
                continue

        if recording and line and line.count(",") >= 5:
            lines.append(line)
            last_sample_time = time.time()

    ser.close()

    if save_csv and len(lines) > 0:
        with open(save_csv, "w", encoding="utf-8") as f:
            f.write("Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS\n")
            for l in lines:
                f.write(l.replace("DATA:", "") + "\n")
        print(f"[+] Đã lưu dữ liệu vào: {save_csv}")

    return parse_csv_file(lines)


def _interpolate_threshold(t_arr, y_arr, target_val, idx_start, idx_end):
    """
    Nội suy tuyến tính thời điểm chạm ngưỡng target_val trong đoạn [idx_start, idx_end].
    Giúp triệt tiêu sai số lượng tử hóa chu kỳ lấy mẫu 10ms.
    """
    for i in range(idx_start + 1, idx_end + 1):
        y_prev = y_arr[i - 1]
        y_curr = y_arr[i]
        if (y_prev <= target_val <= y_curr) or (y_prev >= target_val >= y_curr):
            if y_curr == y_prev:
                return float(t_arr[i])
            frac = (target_val - y_prev) / (y_curr - y_prev)
            return float(t_arr[i - 1] + frac * (t_arr[i] - t_arr[i - 1]))
    return float(t_arr[idx_start])


def analyze_step_response(data):
    """
    Phân tích các thông số động học và nhận dạng hàm truyền động cơ từ dữ liệu đáp ứng xung.
    Chuẩn hóa theo lý thuyết điều khiển tự động (Modern Control Engineering).
    """
    t = data["time_s"]
    pwm = data["pwm"]
    pwm_pct = data["pwm_pct"]
    rpm = data["rpm"]
    rads = data["rads"]

    # 1. Phát hiện thời điểm bật xung (Step Input Start & End)
    step_indices = np.where(np.abs(pwm) > 0)[0]
    if len(step_indices) == 0:
        raise ValueError("Không tìm thấy tín hiệu xung PWM kích hoạt trong dữ liệu!")

    idx_start = step_indices[0]
    idx_end = step_indices[-1]

    t_start = float(t[idx_start])
    t_end = float(t[idx_end])
    step_pwm = float(pwm[idx_start])
    step_pct = float(pwm_pct[idx_start])

    # 2. Tính vận tốc baseline (y_0) trước khi bật xung
    y_0 = float(np.mean(rpm[:idx_start])) if idx_start > 0 else 0.0

    # 3. Tính vận tốc xác lập (Steady State Speed omega_ss)
    # Lấy trung bình ở 35% cuối của khoảng duy trì xung để loại bỏ hoàn toàn quá độ ban đầu
    duration_samples = idx_end - idx_start
    ss_start_idx = idx_start + int(duration_samples * 0.65)
    ss_end_idx = idx_end + 1

    omega_ss_rpm = float(np.mean(rpm[ss_start_idx:ss_end_idx]))
    omega_ss_rads = float(np.mean(rads[ss_start_idx:ss_end_idx]))

    # Biên độ bước nhảy vận tốc
    delta_y = omega_ss_rpm - y_0

    # 4. Tính độ lợi tĩnh (DC Gain K)
    # K = delta_y / Step_Input
    gain_k_unit = delta_y / step_pwm if step_pwm != 0 else 0.0  # RPM / PWM unit
    gain_k_pct = delta_y / step_pct if step_pct != 0 else 0.0    # RPM / % PWM

    # 5. Tính hằng số thời gian (Time Constant tau - 63.2% rule với nội suy tuyến tính)
    target_63 = y_0 + 0.632 * delta_y
    t_tau = _interpolate_threshold(t, rpm, target_63, idx_start, idx_end)
    tau = max(0.001, t_tau - t_start)

    # 6. Tính thời gian tăng (Rise Time tr 10% -> 90% với nội suy tuyến tính)
    target_10 = y_0 + 0.10 * delta_y
    target_90 = y_0 + 0.90 * delta_y

    t_10 = _interpolate_threshold(t, rpm, target_10, idx_start, idx_end)
    t_90 = _interpolate_threshold(t, rpm, target_90, idx_start, idx_end)
    tr = max(0.001, t_90 - t_10)

    # 7. Tính thời gian xác lập (Settling Time ts - dải sai số 2% và 5% chuẩn lý thuyết điều khiển)
    # ts là thời điểm sau đó đáp ứng đi vào và duy trì vĩnh viễn trong dải [omega_ss - delta, omega_ss + delta]
    delta_mag = abs(delta_y)
    band_2pct = 0.02 * delta_mag
    band_5pct = 0.05 * delta_mag

    def _calc_settling_time(band):
        lower = omega_ss_rpm - band
        upper = omega_ss_rpm + band
        for i in range(idx_end, idx_start - 1, -1):
            if rpm[i] < lower or rpm[i] > upper:
                if i == idx_end:
                    return float(t_end - t_start)
                return float(t[i + 1] - t_start)
        return float(t[idx_start] - t_start)

    ts_2pct = _calc_settling_time(band_2pct)
    ts_5pct = _calc_settling_time(band_5pct)

    # 8. Tính độ vọt lố (Overshoot %)
    if delta_y >= 0:
        peak_rpm = float(np.max(rpm[idx_start:idx_end + 1]))
        overshoot_raw = (peak_rpm - omega_ss_rpm) / delta_y * 100.0 if delta_y > 0 else 0.0
    else:
        peak_rpm = float(np.min(rpm[idx_start:idx_end + 1]))
        overshoot_raw = (omega_ss_rpm - peak_rpm) / abs(delta_y) * 100.0 if delta_y < 0 else 0.0

    # Lọc bỏ sai số số học nhỏ (numerical tolerance) do lấy trung bình dải xác lập
    # Đối với hệ bậc 1 thuần túy đơn điệu (< 0.05%), coi như không vọt lố
    overshoot_pct = max(0.0, overshoot_raw) if overshoot_raw >= 0.05 else 0.0

    # 9. Đề xuất thông số PID theo quy tắc Ziegler-Nichols (Bộ điều khiển PI vận tốc)
    dead_time_l = max(0.005, t_10 - t_start)
    kp_pi = (0.9 * tau) / (abs(gain_k_unit) * dead_time_l) if (abs(gain_k_unit) * dead_time_l) > 0 else 1.0
    ti_pi = 3.33 * dead_time_l
    ki_pi = kp_pi / ti_pi if ti_pi > 0 else 0.0

    return {
        "t_start": t_start,
        "t_end": t_end,
        "step_pwm": step_pwm,
        "step_pct": step_pct,
        "y_0": y_0,
        "omega_ss_rpm": omega_ss_rpm,
        "omega_ss_rads": omega_ss_rads,
        "delta_y": delta_y,
        "gain_k_unit": gain_k_unit,
        "gain_k_pct": gain_k_pct,
        "tau": tau,
        "t_tau": t_tau,
        "val_63": target_63,
        "tr": tr,
        "t_10": t_10,
        "t_90": t_90,
        "ts": ts_2pct,
        "ts_2pct": ts_2pct,
        "ts_5pct": ts_5pct,
        "overshoot_pct": overshoot_pct,
        "peak_rpm": peak_rpm,
        "dead_time_l": dead_time_l,
        "kp_suggest": kp_pi,
        "ki_suggest": ki_pi,
    }


def plot_step_response(data, analysis, output_image_path=None):
    """
    Vẽ đồ thị đáp ứng xung đơn vị 2 subplot chuẩn kỹ thuật:
    - Subplot 1: Vận tốc tức thời (RPM), đường xác lập, điểm hằng số thời gian tau, dải Rise Time, dải Settling Time và hộp tham số
    - Subplot 2: Tín hiệu xung đơn vị PWM (%)
    """
    t = data["time_s"]
    pwm_pct = data["pwm_pct"]
    rpm = data["rpm"]

    t_start = analysis["t_start"]
    omega_ss = analysis["omega_ss_rpm"]
    omega_ss_rads = analysis["omega_ss_rads"]
    tau = analysis["tau"]
    t_tau = analysis["t_tau"]
    val_63 = analysis["val_63"]
    tr = analysis["tr"]
    t_10 = analysis["t_10"]
    t_90 = analysis["t_90"]
    ts_2 = analysis["ts_2pct"]
    overshoot = analysis["overshoot_pct"]
    peak_rpm = analysis["peak_rpm"]
    k_unit = analysis["gain_k_unit"]
    step_pct = analysis["step_pct"]
    step_pwm = int(analysis["step_pwm"])

    # Thiết lập kiểu đồ thị kỹ thuật chuyên nghiệp
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 11,
        "axes.labelsize": 13,
        "axes.titlesize": 14,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "figure.titlesize": 15
    })

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9), dpi=300, sharex=True,
                                   gridspec_kw={"height_ratios": [2.5, 1], "hspace": 0.08})

    # ==================== SUBPLOT 1: VẬN TỐC TỨC THỜI ====================
    line_speed, = ax1.plot(t, rpm, color="#0055b3", linewidth=2.2, label="Instantaneous Speed (RPM)")

    # Đường xác lập Steady State
    line_ss = ax1.axhline(y=omega_ss, color="red", linestyle="--", linewidth=1.8,
                          label=f"Steady State $\\omega_{{ss}} = {omega_ss:.1f}$ RPM")

    # Điểm 63.2% - Hằng số thời gian tau
    ax1.plot([t_start, t_tau], [val_63, val_63], color="darkorange", linestyle=":", linewidth=1.8)
    ax1.plot([t_tau, t_tau], [0, val_63], color="darkorange", linestyle=":", linewidth=1.8)
    dot_tau = ax1.plot(t_tau, val_63, marker="o", markersize=8, color="darkorange",
                       linestyle="None", label=f"Time Constant $\\tau = {tau*1000.0:.1f}$ ms (63.2%)")[0]

    # Vùng thời gian tăng Rise Time (10% - 90%)
    span_tr = ax1.axvspan(t_10, t_90, color="#d9edff", alpha=0.45,
                          label=f"Rise Time $t_r = {tr*1000.0:.1f}$ ms (10-90%)")

    # Điểm xác lập Settling Time (2% band)
    t_settled = t_start + ts_2
    line_ts = ax1.axvline(x=t_settled, color="#6f42c1", linestyle="-.", linewidth=1.6,
                          label=f"Settling Time $t_s = {ts_2*1000.0:.1f}$ ms (2% band)")

    # Dải sai số 2% quanh steady state
    band_val = 0.02 * abs(analysis["delta_y"])
    ax1.fill_between([t_start, analysis["t_end"]], omega_ss - band_val, omega_ss + band_val,
                     color="red", alpha=0.08, label="$\\pm 2\\%$ Steady State Band")

    # Hộp tham số nhận dạng động cơ (Motor Step Parameters Box)
    info_text = (
        r"$\bf{Motor\ Step\ Parameters:}$" "\n"
        f"Input Step: {step_pct:.0f}% PWM ({step_pwm}/1000)\n"
        f"Steady Speed $\\omega_{{ss}}$: {omega_ss:.1f} RPM ({omega_ss_rads:.2f} rad/s)\n"
        f"Time Constant $\\tau$: {tau*1000.0:.1f} ms\n"
        f"Rise Time $t_r$: {tr*1000.0:.1f} ms\n"
        f"Settling Time $t_s$ (2%): {ts_2*1000.0:.1f} ms\n"
        f"Overshoot: {overshoot:.1f}%\n"
        f"DC Gain $K$: {k_unit:.4f} RPM/unit\n"
        f"Model: $G(s) = \\frac{{{k_unit:.3f}}}{{{tau:.3f}s + 1}}$\n"
        f"PI Suggest: $K_p={analysis['kp_suggest']:.2f}, K_i={analysis['ki_suggest']:.2f}$"
    )
    # Vị trí hộp tham số và trục Y linh hoạt theo chiều đáp ứng (thuận / nghịch)
    if analysis["delta_y"] < 0:
        y_min = min(peak_rpm * 1.18, omega_ss * 1.25, -5.0) - 5.0
        y_max = max(0.0, float(np.max(rpm))) + 5.0
        info_y = 0.94
        info_va = "top"
    else:
        y_min = min(0.0, float(np.min(rpm))) - 5.0
        y_max = max(peak_rpm * 1.18, omega_ss * 1.25, 5.0) + 5.0
        info_y = 0.06
        info_va = "bottom"

    ax1.set_ylabel(r"$\bf{Motor\ Speed\ [RPM]}$")
    ax1.set_title(r"$\bf{Unit\ Step\ Response\ of\ AGV\ DC\ Motor\ (TB6612\ +\ STM32F103)}$", pad=12)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.set_ylim(y_min, y_max)

    ax1.text(0.98, info_y, info_text, transform=ax1.transAxes, fontsize=10,
             verticalalignment=info_va, horizontalalignment="right",
             bbox=dict(boxstyle="round,pad=0.6", facecolor="white", edgecolor="#cccccc", alpha=0.92, linewidth=1.2))

    ax1.legend(handles=[line_speed, line_ss, dot_tau, span_tr, line_ts], loc="upper left", framealpha=0.9)

    # ==================== SUBPLOT 2: TÍN HIỆU XUNG PWM ====================
    line_pwm, = ax2.plot(t, pwm_pct, color="#2ca02c", linewidth=2.2, label="PWM Duty Cycle (%)")
    ax2.set_ylabel(r"$\bf{PWM\ [\%]}$")
    ax2.set_xlabel(r"$\bf{Time\ [s]}$")

    min_pwm = float(np.min(pwm_pct))
    max_pwm = float(np.max(pwm_pct))
    if min_pwm < -10.0 and max_pwm <= 5.0:
        pwm_min = min(-105.0, min_pwm - 5.0)
        pwm_max = max(5.0, max_pwm + 5.0)
    elif min_pwm < -10.0 and max_pwm > 5.0:
        pwm_min = min(-105.0, min_pwm - 5.0)
        pwm_max = max(105.0, max_pwm + 5.0)
    else:
        pwm_min = min(-5.0, min_pwm - 5.0)
        pwm_max = max(105.0, max_pwm + 5.0)
    ax2.set_ylim(pwm_min, pwm_max)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left", framealpha=0.9)

    plt.subplots_adjust(left=0.08, right=0.98, top=0.94, bottom=0.08, hspace=0.08)

    # Lưu hình ảnh nếu có đường dẫn
    if output_image_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_image_path)), exist_ok=True)
        plt.savefig(output_image_path, dpi=300, bbox_inches="tight")
        print(f"[+] Đã lưu đồ thị phân tích chất lượng cao vào: {output_image_path}")

    return fig


def print_identification_report(analysis):
    """
    In báo cáo nhận dạng hệ thống và gợi ý bộ điều khiển ra terminal.
    """
    print("\n" + "=" * 68)
    print("       SYSTEM IDENTIFICATION REPORT - AGV MOTOR STEP RESPONSE")
    print("=" * 68)
    print(f"  * Input Step Signal:          {analysis['step_pct']:.1f}% PWM ({int(analysis['step_pwm'])}/1000)")
    print(f"  * Steady State Speed:         {analysis['omega_ss_rpm']:.2f} RPM ({analysis['omega_ss_rads']:.3f} rad/s)")
    print(f"  * DC Gain (K):                {analysis['gain_k_unit']:.4f} RPM/unit ({analysis['gain_k_pct']:.3f} RPM/%PWM)")
    print(f"  * Time Constant (tau - 63%):  {analysis['tau']*1000.0:.1f} ms ({analysis['tau']:.4f} s)")
    print(f"  * Rise Time (tr 10-90%):      {analysis['tr']*1000.0:.1f} ms")
    print(f"  * Settling Time (ts 2% band): {analysis['ts_2pct']*1000.0:.1f} ms")
    print(f"  * Settling Time (ts 5% band): {analysis['ts_5pct']*1000.0:.1f} ms")
    print(f"  * Overshoot:                  {analysis['overshoot_pct']:.2f}%")
    print("-" * 68)
    print(f"  * First-Order Transfer Function (FOPTD):")
    print(f"                  {analysis['gain_k_unit']:.4f}")
    print(f"        G(s) = ----------------")
    print(f"                {analysis['tau']:.4f}s + 1")
    print("-" * 68)
    print("  * Suggested Velocity PI Controller Gains:")
    print(f"    - Kp (Proportional Gain):   {analysis['kp_suggest']:.3f}")
    print(f"    - Ki (Integral Gain):       {analysis['ki_suggest']:.3f}")
    print("=" * 68 + "\n")


def main():
    parser = argparse.ArgumentParser(description="AGV Motor Step Response Identification & Plotter")
    parser.add_argument("--data", "-d", default="scripts/step_response_data.csv",
                        help="Đường dẫn file CSV chứa dữ liệu (mặc định: scripts/step_response_data.csv)")
    parser.add_argument("--output", "-o", default="scripts/step_response.png",
                        help="Đường dẫn file ảnh xuất đồ thị (mặc định: scripts/step_response.png)")
    parser.add_argument("--rtt", action="store_true",
                        help="Thu thập dữ liệu trực tiếp từ OpenOCD RTT Server (localhost:9090)")
    parser.add_argument("--serial", default=None,
                        help="Thu thập dữ liệu từ cổng Serial COM (ví dụ: COM3 hoặc /dev/ttyUSB0)")
    parser.add_argument("--no-show", action="store_true",
                        help="Không mở cửa sổ GUI sau khi vẽ (chỉ lưu file ảnh)")

    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_csv = args.data
    if not os.path.isabs(target_csv):
        target_csv = os.path.join(base_dir, target_csv)

    data = None
    if args.rtt:
        data = capture_from_rtt(save_csv=target_csv)
    elif args.serial:
        data = capture_from_serial(port=args.serial, save_csv=target_csv)
    else:
        data = parse_csv_file(target_csv)

    # Phân tích nhận dạng
    analysis = analyze_step_response(data)

    # In kết quả nhận dạng ra terminal
    print_identification_report(analysis)

    # Vẽ và lưu đồ thị
    out_img = args.output
    if not os.path.isabs(out_img):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out_img = os.path.join(base_dir, out_img)

    plot_step_response(data, analysis, output_image_path=out_img)

    if not args.no_show:
        try:
            plt.show()
        except Exception:
            pass


if __name__ == "__main__":
    main()
