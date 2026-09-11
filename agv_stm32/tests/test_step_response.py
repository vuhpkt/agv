#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for AGV Motor Step Response Identification & Analysis
"""

import os
import sys
import unittest
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Chạy headless không cần màn hình GUI
import matplotlib.pyplot as plt

# Thêm thư mục scripts vào sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts")))

from plot_step_response import parse_csv_file, analyze_step_response, plot_step_response


class TestStepResponseAnalysis(unittest.TestCase):

    def test_parse_data_with_prefix(self):
        """Kiểm tra parser xử lý đúng dữ liệu stream có tiền tố DATA: từ RTT/Serial"""
        sample_stream = [
            "========================================",
            "  UNIT STEP RESPONSE TEST STARTED",
            "========================================",
            "DATA_START",
            "Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS",
            "DATA:0,0.0000,0,0.0,0,0.00,0.000",
            "DATA:10,0.0100,0,0.0,0,0.00,0.000",
            "DATA:200,0.2000,800,80.0,15,68.18,7.140",
            "DATA:210,0.2100,800,80.0,25,113.64,11.900",
            "DATA:220,0.2200,800,80.0,38,172.73,18.088",
            "DATA_END",
            "[STEP_TEST] TEST_PASS | Peak: 172.7 RPM"
        ]
        data = parse_csv_file(sample_stream)
        self.assertEqual(len(data["time_s"]), 5)
        self.assertEqual(data["pwm"][0], 0.0)
        self.assertEqual(data["pwm"][2], 800.0)
        self.assertAlmostEqual(data["rpm"][2], 68.18, places=2)
        self.assertAlmostEqual(data["rads"][2], 7.140, places=3)

    def test_analytical_first_order_identification(self):
        """
        Tạo hàm truyền giải tích bậc 1: G(s) = K / (tau*s + 1)
        với K = 0.225 RPM/unit, PWM = 800 (80%), tau = 0.120s, t_start = 0.200s, dt = 10ms.
        Kiểm tra độ chính xác của thuật toán nhận dạng hệ thống.
        """
        k_true = 0.225
        tau_true = 0.120
        pwm_target = 800.0
        pwm_pct = 80.0
        y_ss_true = k_true * pwm_target  # 180.0 RPM
        t_start = 0.200
        t_end = 2.200
        dt = 0.010

        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.zeros_like(t_arr)
        rpm_arr = np.zeros_like(t_arr)

        for i, t in enumerate(t_arr):
            if t_start <= t < t_end:
                pwm_arr[i] = pwm_target
                t_elapsed = t - t_start
                rpm_arr[i] = y_ss_true * (1.0 - np.exp(-t_elapsed / tau_true))
            elif t >= t_end:
                pwm_arr[i] = 0.0
                rpm_arr[i] = y_ss_true * np.exp(-(t - t_end) / tau_true)

        rads_arr = rpm_arr * (2.0 * np.pi / 60.0)
        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": (pwm_arr * 100.0) / 1000.0,
            "delta": (rpm_arr * 1320 * dt / 60.0).astype(int),
            "rpm": rpm_arr,
            "rads": rads_arr,
        }

        analysis = analyze_step_response(data)

        # 1. Kiểm tra vận tốc xác lập (sai số < 0.5%)
        self.assertAlmostEqual(analysis["omega_ss_rpm"], y_ss_true, delta=0.5)

        # 2. Kiểm tra độ lợi tĩnh K (sai số < 0.5%)
        self.assertAlmostEqual(analysis["gain_k_unit"], k_true, delta=0.002)

        # 3. Kiểm tra hằng số thời gian tau (nhờ nội suy tuyến tính, sai số < 3ms)
        self.assertAlmostEqual(analysis["tau"], tau_true, delta=0.003)

        # 4. Kiểm tra Rise Time giải tích: tr = tau * ln(9) = 0.120 * 2.1972 = 0.2637s
        tr_theoretical = tau_true * np.log(9.0)
        self.assertAlmostEqual(analysis["tr"], tr_theoretical, delta=0.005)

        # 5. Kiểm tra Settling Time 2% giải tích: ts = tau * ln(1/0.02) = 0.120 * 3.912 = 0.469s
        ts_theoretical = tau_true * np.log(1.0 / 0.02)
        self.assertAlmostEqual(analysis["ts_2pct"], ts_theoretical, delta=0.015)

        # 6. Kiểm tra độ vọt lố (đáp ứng bậc 1 thuần túy overshoot = 0%)
        self.assertEqual(analysis["overshoot_pct"], 0.0)

        # 7. Kiểm tra thông số PI gợi ý hợp lệ
        self.assertGreater(analysis["kp_suggest"], 0.0)
        self.assertGreater(analysis["ki_suggest"], 0.0)

    def test_reverse_direction_step_response(self):
        """Kiểm tra thuật toán xử lý chính xác đáp ứng xung chiều nghịch (negative PWM)"""
        dt = 0.010
        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.zeros_like(t_arr)
        rpm_arr = np.zeros_like(t_arr)
        t_start = 0.200
        t_end = 2.200
        tau_true = 0.100
        y_ss_true = -180.0

        for i, t in enumerate(t_arr):
            if t_start <= t < t_end:
                pwm_arr[i] = -800.0  # -80% PWM
                rpm_arr[i] = y_ss_true * (1.0 - np.exp(-(t - t_start) / tau_true))
            elif t >= t_end:
                pwm_arr[i] = 0.0
                rpm_arr[i] = y_ss_true * np.exp(-(t - t_end) / tau_true)

        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": pwm_arr / 10.0,
            "delta": np.zeros_like(t_arr, dtype=int),
            "rpm": rpm_arr,
            "rads": rpm_arr * (2.0 * np.pi / 60.0),
        }

        analysis = analyze_step_response(data)
        self.assertAlmostEqual(analysis["omega_ss_rpm"], -180.0, delta=1.0)
        self.assertAlmostEqual(analysis["gain_k_unit"], 0.225, delta=0.005)
        self.assertAlmostEqual(analysis["tau"], 0.100, delta=0.005)

    def test_missing_pwm_raises_error(self):
        """Dữ liệu không có xung PWM phải báo lỗi ValueError rõ ràng"""
        t_arr = np.linspace(0.0, 1.0, 50)
        data = {
            "time_s": t_arr,
            "pwm": np.zeros_like(t_arr),
            "pwm_pct": np.zeros_like(t_arr),
            "delta": np.zeros_like(t_arr, dtype=int),
            "rpm": np.zeros_like(t_arr),
            "rads": np.zeros_like(t_arr),
        }
        with self.assertRaises(ValueError):
            analyze_step_response(data)

    def test_empty_lines_raises_error(self):
        """Nguồn dữ liệu rỗng phải báo lỗi ValueError"""
        with self.assertRaises(ValueError):
            parse_csv_file([])

    def test_plot_step_response_forward(self):
        """Kiểm tra hàm vẽ đồ thị plot_step_response tạo figure và lưu file thành công"""
        dt = 0.010
        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), 800.0, 0.0)
        rpm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), 180.0 * (1.0 - np.exp(-(t_arr - 0.2) / 0.12)), 0.0)
        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": pwm_arr / 10.0,
            "delta": (rpm_arr * 1320 * dt / 60.0).astype(int),
            "rpm": rpm_arr,
            "rads": rpm_arr * (2.0 * np.pi / 60.0),
        }
        analysis = analyze_step_response(data)
        out_path = os.path.join(os.path.dirname(__file__), "test_forward_out.png")
        try:
            fig = plot_step_response(data, analysis, output_image_path=out_path)
            self.assertIsNotNone(fig)
            self.assertTrue(os.path.exists(out_path))
            self.assertGreater(os.path.getsize(out_path), 10000)
            # Kiểm tra trục Y đồ thị thuận
            ax1, ax2 = fig.get_axes()
            y_lim = ax1.get_ylim()
            self.assertLessEqual(y_lim[0], 0.0)
            self.assertGreater(y_lim[1], 180.0)
            plt.close(fig)
        finally:
            if os.path.exists(out_path):
                os.remove(out_path)

    def test_plot_step_response_reverse(self):
        """Kiểm tra hàm vẽ đồ thị plot_step_response cho xung nghịch (negative PWM & RPM)"""
        dt = 0.010
        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), -800.0, 0.0)
        rpm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), -180.0 * (1.0 - np.exp(-(t_arr - 0.2) / 0.1)), 0.0)
        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": pwm_arr / 10.0,
            "delta": np.zeros_like(t_arr, dtype=int),
            "rpm": rpm_arr,
            "rads": rpm_arr * (2.0 * np.pi / 60.0),
        }
        analysis = analyze_step_response(data)
        out_path = os.path.join(os.path.dirname(__file__), "test_reverse_out.png")
        try:
            fig = plot_step_response(data, analysis, output_image_path=out_path)
            self.assertIsNotNone(fig)
            self.assertTrue(os.path.exists(out_path))
            # Trục Y tốc độ nghịch phải có dải âm rõ ràng và trần dương nhỏ
            ax1, ax2 = fig.get_axes()
            y_lim = ax1.get_ylim()
            self.assertLess(y_lim[0], -180.0)
            self.assertLessEqual(y_lim[1], 15.0)
            # Trục PWM nghịch phải nằm dải âm
            pwm_lim = ax2.get_ylim()
            self.assertLessEqual(pwm_lim[0], -80.0)
            plt.close(fig)
        finally:
            if os.path.exists(out_path):
                os.remove(out_path)

    def test_stalled_motor_response(self):
        """Kiểm tra trường hợp kẹt động cơ (PWM kích hoạt nhưng tốc độ = 0 RPM), không được lỗi chia 0"""
        dt = 0.010
        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), 800.0, 0.0)
        rpm_arr = np.zeros_like(t_arr)
        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": pwm_arr / 10.0,
            "delta": np.zeros_like(t_arr, dtype=int),
            "rpm": rpm_arr,
            "rads": np.zeros_like(t_arr),
        }
        analysis = analyze_step_response(data)
        self.assertEqual(analysis["omega_ss_rpm"], 0.0)
        self.assertEqual(analysis["gain_k_unit"], 0.0)
        self.assertEqual(analysis["overshoot_pct"], 0.0)
        self.assertEqual(analysis["ts_2pct"], 0.0)

    def test_parse_stream_with_rtt_banner_and_logs(self):
        """Kiểm tra parser lọc sạch banner SEGGER RTT, heartbeat và log xen kẽ"""
        complex_stream = [
            "\r\n\x1b[1;32m========================================",
            "  STM32 AGV System Initialized",
            "========================================\x1b[0m",
            "[BOOT] Peripherals & Drivers Ready",
            "[HB] Heartbeat #1 (Tick: 1000 ms)",
            "[ENC] Left: 0 (d:0) | Right: 0 (d:0)",
            "DATA_START",
            "Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS",
            "DATA:0,0.0000,0,0.0,0,0.00,0.000",
            "DATA:10,0.0100,0,0.0,0,0.00,0.000",
            "DATA:200,0.2000,800,80.0,15,68.18,7.140",
            "DATA:210,0.2100,800,80.0,25,113.64,11.900",
            "DATA_END",
            "[STEP_TEST] TEST_PASS | Peak: 113.6 RPM | Steady: 113.6 RPM | Ticks: 40 | Samples: 4",
            "[HB] Heartbeat #2 (Tick: 2000 ms)"
        ]
        data = parse_csv_file(complex_stream)
        self.assertEqual(len(data["time_s"]), 4)
        self.assertEqual(data["time_s"][0], 0.0)
        self.assertEqual(data["time_s"][-1], 0.21)
        self.assertEqual(data["pwm"][2], 800.0)

    def test_noisy_encoder_step_response(self):
        """Kiểm tra độ bền vững của thuật toán nhận dạng khi tín hiệu encoder có nhiễu Gaussian nhẹ"""
        np.random.seed(42)
        dt = 0.010
        t_arr = np.arange(0.0, 2.5, dt)
        pwm_arr = np.where((t_arr >= 0.2) & (t_arr < 2.2), 800.0, 0.0)
        clean_rpm = np.where((t_arr >= 0.2) & (t_arr < 2.2), 180.0 * (1.0 - np.exp(-(t_arr - 0.2) / 0.12)), 0.0)
        # Thêm nhiễu +-1.0 RPM
        noise = np.random.normal(0.0, 0.8, size=len(t_arr))
        rpm_noisy = clean_rpm + noise

        data = {
            "time_s": t_arr,
            "pwm": pwm_arr,
            "pwm_pct": pwm_arr / 10.0,
            "delta": np.zeros_like(t_arr, dtype=int),
            "rpm": rpm_noisy,
            "rads": rpm_noisy * (2.0 * np.pi / 60.0),
        }
        analysis = analyze_step_response(data)
        self.assertAlmostEqual(analysis["omega_ss_rpm"], 180.0, delta=2.0)
        self.assertAlmostEqual(analysis["gain_k_unit"], 0.225, delta=0.01)
        self.assertAlmostEqual(analysis["tau"], 0.120, delta=0.015)


if __name__ == "__main__":
    unittest.main()


