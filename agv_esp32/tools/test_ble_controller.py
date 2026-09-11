#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for AGV BLE Wireless Controller & Diagnostics Suite.
Tests protocol parsing, regex extractors, calibration statistics, and command translation.
"""

import unittest
import os
import sys

# Ensure tools directory is on sys.path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from agv_ble_controller import (
    render_bar,
    parse_line_telemetry,
    parse_step_telemetry,
    CalibrationStats,
    validate_calibration,
    generate_header_content,
    translate_cli_command,
    AGVBleClient,
)


class TestBleControllerParsers(unittest.TestCase):
    def test_render_bar(self):
        # 0% -> all empty blocks
        b0 = render_bar(0, max_val=1000, width=20)
        self.assertEqual(b0, "░" * 20)

        # 100% -> all filled blocks
        b100 = render_bar(1000, max_val=1000, width=20)
        self.assertEqual(b100, "█" * 20)

        # 50% -> 10 filled, 10 empty
        b50 = render_bar(500, max_val=1000, width=20)
        self.assertEqual(b50, "█" * 10 + "░" * 10)

        # Clamping
        b_neg = render_bar(-100, max_val=1000, width=20)
        self.assertEqual(b_neg, "░" * 20)
        b_over = render_bar(1500, max_val=1000, width=20)
        self.assertEqual(b_over, "█" * 20)

    def test_parse_line_telemetry_valid(self):
        sample = "DATA:LINE,1250,2450,3820,3910,1650,210,600,950,980,400,50,-150,1"
        res = parse_line_telemetry(sample)
        self.assertIsNotNone(res)
        self.assertEqual(res["elapsed_ms"], 1250)
        self.assertEqual(res["raw"], [2450, 3820, 3910, 1650, 210])
        self.assertEqual(res["norm"], [600, 950, 980, 400, 50])
        self.assertEqual(res["err"], -150)
        self.assertTrue(res["line_seen"])

    def test_parse_line_telemetry_no_line(self):
        sample = "DATA:LINE,500,100,120,110,95,105,10,15,12,8,14,0,0"
        res = parse_line_telemetry(sample)
        self.assertIsNotNone(res)
        self.assertEqual(res["elapsed_ms"], 500)
        self.assertFalse(res["line_seen"])

    def test_parse_line_telemetry_invalid(self):
        self.assertIsNone(parse_line_telemetry("INVALID LINE"))
        self.assertIsNone(parse_line_telemetry("DATA:LINE,1,2,3"))
        self.assertIsNone(parse_line_telemetry("[STM32 LOG] Booting"))

    def test_parse_step_telemetry_valid(self):
        sample = "DATA:250,0.250,600,60.0,145,82.4,8.63"
        res = parse_step_telemetry(sample)
        self.assertIsNotNone(res)
        self.assertEqual(res["elapsed_ms"], 250)
        self.assertAlmostEqual(res["time_s"], 0.250)
        self.assertEqual(res["pwm"], 600)
        self.assertAlmostEqual(res["pwm_pct"], 60.0)
        self.assertEqual(res["delta_ticks"], 145)
        self.assertAlmostEqual(res["rpm"], 82.4)
        self.assertAlmostEqual(res["rads"], 8.63)

    def test_parse_step_telemetry_negative(self):
        sample = "DATA:500,0.500,-600,-60.0,-140,-80.1,-8.38"
        res = parse_step_telemetry(sample)
        self.assertIsNotNone(res)
        self.assertEqual(res["pwm"], -600)
        self.assertAlmostEqual(res["rpm"], -80.1)

    def test_parse_step_telemetry_invalid(self):
        self.assertIsNone(parse_step_telemetry("DATA:LINE,1,2,3"))
        self.assertIsNone(parse_step_telemetry("DATA:not_a_number"))

    def test_parse_step_telemetry_missing_rads(self):
        # When rads (7th group) is omitted, it should compute rads from rpm
        sample = "DATA:300,0.300,500,50.0,120,60.0"
        res = parse_step_telemetry(sample)
        self.assertIsNotNone(res)
        self.assertEqual(res["elapsed_ms"], 300)
        self.assertAlmostEqual(res["rpm"], 60.0)
        # 60 RPM = 2 * pi rad/s ~= 6.283185
        self.assertAlmostEqual(res["rads"], 6.283185, places=3)

    def test_step_response_peak_rpm_calculation(self):
        # Positive PWM: peak speed is positive max
        rpms_forward = [0.0, 20.5, 65.2, 85.4, 82.0, 81.5]
        peak_fwd = max(rpms_forward, key=abs)
        self.assertEqual(peak_fwd, 85.4)

        # Reverse PWM: peak speed magnitude must be -88.6, NOT -10.2
        rpms_reverse = [0.0, -10.2, -45.0, -88.6, -81.0, -80.5]
        peak_rev = max(rpms_reverse, key=abs)
        self.assertEqual(peak_rev, -88.6)

    def test_chunked_ble_notification_reassembly(self):
        # Simulate fragmented BLE chunks arriving across arbitrary boundaries
        client = AGVBleClient(device_name="TEST_AGV")
        received_lines = []
        client.add_line_listener(lambda line: received_lines.append(line))

        # Chunk 1: partial DATA:LINE
        client._on_notification(None, bytearray(b"DATA:LINE,1250,2450,3820,"))
        self.assertEqual(len(received_lines), 0)

        # Chunk 2: remainder of DATA:LINE + partial [ACK]
        client._on_notification(None, bytearray(b"3910,1650,210,600,950,980,400,50,-150,1\r\n[ACK] MsgID: 0x"))
        self.assertEqual(len(received_lines), 1)
        self.assertEqual(received_lines[0], "DATA:LINE,1250,2450,3820,3910,1650,210,600,950,980,400,50,-150,1")

        # Chunk 3: remainder of [ACK]
        client._on_notification(None, bytearray(b"20, Status: OK (Code: 0)\r\n"))
        self.assertEqual(len(received_lines), 2)
        self.assertEqual(received_lines[1], "[ACK] MsgID: 0x20, Status: OK (Code: 0)")


class TestCalibrationLogic(unittest.TestCase):
    def test_calibration_stats_update(self):
        stats = CalibrationStats(num_sensors=5)
        self.assertEqual(stats.sample_count, 0)

        stats.update([500, 600, 700, 800, 900])
        stats.update([200, 300, 400, 1200, 1500])

        self.assertEqual(stats.sample_count, 2)
        self.assertEqual(stats.min_raw, [200, 300, 400, 800, 900])
        self.assertEqual(stats.max_raw, [500, 600, 700, 1200, 1500])

    def test_validate_calibration_pass(self):
        min_raw = [200, 250, 300, 220, 210]
        max_raw = [3200, 3400, 3500, 3300, 3100]
        is_valid, errors = validate_calibration(min_raw, max_raw, min_dynamic_range=100)
        self.assertTrue(is_valid)
        self.assertEqual(len(errors), 0)

    def test_validate_calibration_fail_delta(self):
        min_raw = [200, 250, 300, 220, 210]
        max_raw = [250, 3400, 3500, 3300, 3100] # Channel 0 delta = 50 < 100
        is_valid, errors = validate_calibration(min_raw, max_raw, min_dynamic_range=100)
        self.assertFalse(is_valid)
        self.assertTrue(any("Dynamic range too low" in e for e in errors))

    def test_validate_calibration_fail_min_ge_max(self):
        min_raw = [2000, 250, 300, 220, 210]
        max_raw = [1000, 3400, 3500, 3300, 3100] # Channel 0 min > max
        is_valid, errors = validate_calibration(min_raw, max_raw, min_dynamic_range=100)
        self.assertFalse(is_valid)
        self.assertTrue(any("Min ADC" in e for e in errors))

    def test_validate_calibration_fail_out_of_bounds(self):
        # Negative reading (< 0)
        is_valid, errors = validate_calibration([-1, 250, 300, 220, 210], [3200, 3400, 3500, 3300, 3100])
        self.assertFalse(is_valid)
        self.assertTrue(any("out of bounds" in e for e in errors))

        # Overshoot reading (> 4095)
        is_valid, errors = validate_calibration([200, 250, 300, 220, 210], [4096, 3400, 3500, 3300, 3100])
        self.assertFalse(is_valid)
        self.assertTrue(any("out of bounds" in e for e in errors))

    def test_calibration_stats_invalid_length(self):
        stats = CalibrationStats(num_sensors=5)
        stats.update([100, 200, 300]) # only 3 values instead of 5
        self.assertEqual(stats.sample_count, 0)
        self.assertEqual(stats.min_raw, [4095] * 5)
        self.assertEqual(stats.max_raw, [0] * 5)

    def test_generate_header_content(self):
        min_raw = [100, 110, 120, 130, 140]
        max_raw = [3000, 3100, 3200, 3300, 3400]
        header = generate_header_content(min_raw, max_raw)
        self.assertIn("#define APP_LINE_CALIB_MIN_RAW   { 100, 110, 120, 130, 140 }", header)
        self.assertIn("#define APP_LINE_CALIB_MAX_RAW   { 3000, 3100, 3200, 3300, 3400 }", header)
        self.assertIn("#ifndef APP_LINE_CALIB_DEFAULTS_H", header)


class TestCommandTranslation(unittest.TestCase):
    def test_simple_commands(self):
        self.assertEqual(translate_cli_command("ping"), "PING")
        self.assertEqual(translate_cli_command("STOP"), "STOP")
        self.assertEqual(translate_cli_command("emg"), "STOP")
        self.assertEqual(translate_cli_command("get_enc"), "GET_ENC")
        self.assertEqual(translate_cli_command("reset_enc"), "RESET_ENC")
        self.assertEqual(translate_cli_command("route_start"), "ROUTE_START")
        self.assertEqual(translate_cli_command("route_cancel"), "ROUTE_CANCEL")
        self.assertEqual(translate_cli_command("route_test"), "ROUTE_TEST")
        self.assertEqual(translate_cli_command("help"), "HELP")

    def test_parameterized_commands(self):
        self.assertEqual(translate_cli_command("step L 600 1500"), "STEP_TEST L 600 1500")
        self.assertEqual(translate_cli_command("step_test R 500 2000"), "STEP_TEST R 500 2000")
        self.assertEqual(translate_cli_command("set_speed 500 -500"), "SET_SPEED 500 -500")
        self.assertEqual(translate_cli_command("set_rpm 80.0 80.0"), "SET_RPM 80.0 80.0")
        self.assertEqual(translate_cli_command("pi_tune 1.5 0.2 0.05"), "PI_TUNE 1.5 0.2 0.05")
        self.assertEqual(translate_cli_command("monitor_ir 10000"), "MONITOR_IR 10000")
        self.assertEqual(translate_cli_command("route_load 80 F,R,F,S"), "ROUTE_LOAD 80 F,R,F,S")

    def test_empty_and_whitespace(self):
        self.assertEqual(translate_cli_command(""), "")
        self.assertEqual(translate_cli_command("   "), "")
        self.assertEqual(translate_cli_command("\t\n"), "")

    def test_new_route_aliases(self):
        self.assertEqual(translate_cli_command("load 80.0 F,R"), "ROUTE_LOAD 80.0 F,R")
        self.assertEqual(translate_cli_command("start"), "ROUTE_START")
        self.assertEqual(translate_cli_command("cancel"), "ROUTE_CANCEL")

    def test_unknown_command_passthrough(self):
        self.assertEqual(translate_cli_command("custom_diag 123"), "CUSTOM_DIAG 123")


if __name__ == "__main__":
    unittest.main()
