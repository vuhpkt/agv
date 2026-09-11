#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Comms Protocol Service (Task 4)
Tests parsing of commands: PING, SET_LED, SET_SPEED, STOP, BRAKE, GET_ENC,
RESET_ENC, STEP_TEST, RPM, PI_TUNE, direct speed numbers, and edge cases.
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class CommsProtocolParserReference:
    """
    Python reference model for Services/Src/comms_protocol.c
    """
    @staticmethod
    def parse_line(line: str):
        line = line.strip()
        if not line:
            return {"type": "NONE"}

        parts = line.split()
        cmd = parts[0].upper()

        if cmd == "PING":
            return {"type": "PING"}
        elif cmd == "SET_LED":
            # Argument can be 'SET_LED 1' or 'SET_LED=1'
            val_str = line[7:].replace("=", "").strip()
            state = int(val_str) if val_str.isdigit() else 0
            return {"type": "SET_LED", "state": state == 1}
        elif cmd == "SET_SPEED":
            if len(parts) >= 3:
                return {"type": "SET_SPEED", "left": int(parts[1]), "right": int(parts[2])}
            return {"type": "ERROR", "msg": "INVALID_SPEED_ARGS"}
        elif cmd in ("STOP", "STEP_STOP"):
            return {"type": "STOP"}
        elif cmd == "BRAKE":
            return {"type": "BRAKE"}
        elif cmd == "GET_ENC":
            return {"type": "GET_ENC"}
        elif cmd == "RESET_ENC":
            return {"type": "RESET_ENC"}
        elif cmd in ("STEP_TEST", "MOT"):
            # STEP_TEST / MOT [LEFT|RIGHT|A|B|0|1] [pwm] [duration_ms]
            motor = 0  # Left / A
            pwm = 800
            duration = 2000
            args = parts[1:]
            if args:
                idx = 0
                first_arg = args[0].upper()
                if first_arg in ("LEFT", "RIGHT", "A", "B", "0", "1"):
                    if first_arg in ("RIGHT", "B", "1"):
                        motor = 1
                    else:
                        motor = 0
                    idx += 1
                if idx < len(args):
                    try:
                        pwm = int(args[idx])
                        idx += 1
                    except ValueError:
                        pass
                if idx < len(args):
                    try:
                        duration = int(args[idx])
                    except ValueError:
                        pass
            return {"type": "STEP_TEST", "motor": motor, "pwm": pwm, "duration_ms": duration}
        elif cmd == "RPM":
            if len(parts) >= 3:
                return {"type": "RPM", "left": float(parts[1]), "right": float(parts[2])}
            elif len(parts) == 2:
                v = float(parts[1])
                return {"type": "RPM", "left": v, "right": v}
            return {"type": "ERROR", "msg": "INVALID_RPM_ARGS"}
        elif cmd == "PI_STATUS":
            return {"type": "PI_STATUS"}
        elif cmd == "PI_TUNE":
            if len(parts) >= 3:
                kp = float(parts[1]) / 100.0
                ki = float(parts[2]) / 100.0
                kff = float(parts[3]) / 100.0 if len(parts) >= 4 else 3.705
                return {"type": "PI_TUNE", "kp": kp, "ki": ki, "k_ff": kff}
            return {"type": "ERROR", "msg": "INVALID_TUNE_ARGS"}
        elif cmd == "LINE_TEST":
            mode = 0
            dur = 3000
            rpm = 80.0
            if len(parts) >= 2:
                sub = parts[1].upper()
                if sub == "STOP":
                    return {"type": "LINE_TEST", "mode": 3, "duration_ms": dur, "base_rpm": rpm}
                elif sub == "CALIB":
                    mode = 1
                    if len(parts) >= 3:
                        dur = int(parts[2])
                elif sub in ("TRACK", "RUN"):
                    mode = 2
                    if len(parts) >= 3:
                        rpm = float(parts[2])
                    if len(parts) >= 4:
                        dur = int(parts[3])
                elif sub == "SENSOR":
                    mode = 0
                    if len(parts) >= 3:
                        dur = int(parts[2])
                else:
                    try:
                        dur = int(parts[1])
                    except ValueError:
                        pass
            return {"type": "LINE_TEST", "mode": mode, "duration_ms": dur, "base_rpm": rpm}
        else:
            # Check direct speed syntax "<left> <right>"
            if len(parts) == 2:
                try:
                    l = int(parts[0])
                    r = int(parts[1])
                    return {"type": "SET_SPEED", "left": l, "right": r}
                except ValueError:
                    pass
            return {"type": "UNKNOWN", "raw": line}


class TestCommsProtocol(unittest.TestCase):

    def test_protocol_files_exist(self):
        """Task 4: Services/Inc/comms_protocol.h and Services/Src/comms_protocol.c must exist"""
        header = os.path.join(REPO_ROOT, "Services", "Inc", "comms_protocol.h")
        src = os.path.join(REPO_ROOT, "Services", "Src", "comms_protocol.c")
        self.assertTrue(os.path.exists(header), "Services/Inc/comms_protocol.h does not exist")
        self.assertTrue(os.path.exists(src), "Services/Src/comms_protocol.c does not exist")

    def test_protocol_in_cmake(self):
        """Task 4: CMakeLists.txt must include Services/Src/comms_protocol.c"""
        cmake_path = os.path.join(REPO_ROOT, "CMakeLists.txt")
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("Services/Src/comms_protocol.c", content)

    def test_parse_basic_commands(self):
        """Kiểm tra phân tích các lệnh cơ bản: PING, STOP, BRAKE, GET_ENC, RESET_ENC"""
        parser = CommsProtocolParserReference()
        self.assertEqual(parser.parse_line("PING")["type"], "PING")
        self.assertEqual(parser.parse_line("  ping  ")["type"], "PING")
        self.assertEqual(parser.parse_line("STOP")["type"], "STOP")
        self.assertEqual(parser.parse_line("STEP_STOP")["type"], "STOP")
        self.assertEqual(parser.parse_line("BRAKE")["type"], "BRAKE")
        self.assertEqual(parser.parse_line("GET_ENC")["type"], "GET_ENC")
        self.assertEqual(parser.parse_line("RESET_ENC")["type"], "RESET_ENC")

    def test_parse_speed_and_rpm(self):
        """Kiểm tra phân tích lệnh tốc độ và RPM"""
        parser = CommsProtocolParserReference()

        # SET_SPEED
        res = parser.parse_line("SET_SPEED 500 -500")
        self.assertEqual(res["type"], "SET_SPEED")
        self.assertEqual(res["left"], 500)
        self.assertEqual(res["right"], -500)

        # Cú pháp nhập trực tiếp 2 số: "400 400"
        res_direct = parser.parse_line("400 -400")
        self.assertEqual(res_direct["type"], "SET_SPEED")
        self.assertEqual(res_direct["left"], 400)
        self.assertEqual(res_direct["right"], -400)

        # RPM cả 2 bánh giống nhau
        res_rpm1 = parser.parse_line("RPM 120")
        self.assertEqual(res_rpm1["type"], "RPM")
        self.assertEqual(res_rpm1["left"], 120.0)
        self.assertEqual(res_rpm1["right"], 120.0)

        # RPM từng bánh
        res_rpm2 = parser.parse_line("rpm -150 150")
        self.assertEqual(res_rpm2["type"], "RPM")
        self.assertEqual(res_rpm2["left"], -150.0)
        self.assertEqual(res_rpm2["right"], 150.0)

    def test_parse_step_test(self):
        """Kiểm tra phân tích lệnh STEP_TEST"""
        parser = CommsProtocolParserReference()

        # Mặc định
        res = parser.parse_line("STEP_TEST")
        self.assertEqual(res["type"], "STEP_TEST")
        self.assertEqual(res["motor"], 0)
        self.assertEqual(res["pwm"], 800)

        # Chọn motor phải, PWM, thời gian
        res_r = parser.parse_line("STEP_TEST RIGHT 600 1500")
        self.assertEqual(res_r["type"], "STEP_TEST")
        self.assertEqual(res_r["motor"], 1)
        self.assertEqual(res_r["pwm"], 600)
        self.assertEqual(res_r["duration_ms"], 1500)

    def test_parse_edge_cases(self):
        """Kiểm tra các trường hợp ngoại lệ: chuỗi rỗng, khoảng trắng, lệnh sai"""
        parser = CommsProtocolParserReference()
        self.assertEqual(parser.parse_line("")["type"], "NONE")
        self.assertEqual(parser.parse_line("   ")["type"], "NONE")
        self.assertEqual(parser.parse_line("SOME_WEIRD_STRING")["type"], "UNKNOWN")

    def test_line_tracking_commands_in_header_and_parser(self):
        """Services/Inc/comms_protocol.h must declare AGV_MSG_DIAG_LINE_TEST"""
        header_path = os.path.join(REPO_ROOT, "Services", "Inc", "comms_protocol.h")
        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("AGV_MSG_DIAG_LINE_TEST", content, "comms_protocol.h missing AGV_MSG_DIAG_LINE_TEST")

    def test_parse_line_test_commands(self):
        """Kiểm tra phân tích cú pháp lệnh LINE_TEST cho mảng cảm biến IR"""
        parser = CommsProtocolParserReference()

        # Mặc định: SENSOR monitor, 3000ms, 80 RPM
        res_def = parser.parse_line("LINE_TEST")
        self.assertEqual(res_def["type"], "LINE_TEST")
        self.assertEqual(res_def["mode"], 0)
        self.assertEqual(res_def["duration_ms"], 3000)

        # Chế độ SENSOR có thời lượng
        res_sensor = parser.parse_line("line_test sensor 2000")
        self.assertEqual(res_sensor["type"], "LINE_TEST")
        self.assertEqual(res_sensor["mode"], 0)
        self.assertEqual(res_sensor["duration_ms"], 2000)

        # Chế độ CALIB
        res_calib = parser.parse_line("LINE_TEST CALIB 4000")
        self.assertEqual(res_calib["type"], "LINE_TEST")
        self.assertEqual(res_calib["mode"], 1)
        self.assertEqual(res_calib["duration_ms"], 4000)

        # Chế độ TRACK / RUN
        res_track = parser.parse_line("LINE_TEST RUN 100 5000")
        self.assertEqual(res_track["type"], "LINE_TEST")
        self.assertEqual(res_track["mode"], 2)
        self.assertAlmostEqual(res_track["base_rpm"], 100.0)
        self.assertEqual(res_track["duration_ms"], 5000)

        # Chế độ STOP
        res_stop = parser.parse_line("LINE_TEST STOP")
        self.assertEqual(res_stop["type"], "LINE_TEST")
        self.assertEqual(res_stop["mode"], 3)

    def test_parse_step_test_and_mot_ab(self):
        """Kiểm tra phân tích lệnh STEP_TEST và MOT với tham số A/B"""
        parser = CommsProtocolParserReference()

        # STEP_TEST với A / B
        res_a = parser.parse_line("STEP_TEST A 400 2000")
        self.assertEqual(res_a["type"], "STEP_TEST")
        self.assertEqual(res_a["motor"], 0)
        self.assertEqual(res_a["pwm"], 400)
        self.assertEqual(res_a["duration_ms"], 2000)

        res_b_neg = parser.parse_line("step_test b -400 1500")
        self.assertEqual(res_b_neg["type"], "STEP_TEST")
        self.assertEqual(res_b_neg["motor"], 1)
        self.assertEqual(res_b_neg["pwm"], -400)
        self.assertEqual(res_b_neg["duration_ms"], 1500)

        # Lệnh ngắn MOT <A|B> <pwm> [duration]
        res_mot_a = parser.parse_line("MOT A 400")
        self.assertEqual(res_mot_a["type"], "STEP_TEST")
        self.assertEqual(res_mot_a["motor"], 0)
        self.assertEqual(res_mot_a["pwm"], 400)
        self.assertEqual(res_mot_a["duration_ms"], 2000)

        res_mot_b = parser.parse_line("mot b -500 1200")
        self.assertEqual(res_mot_b["type"], "STEP_TEST")
        self.assertEqual(res_mot_b["motor"], 1)
        self.assertEqual(res_mot_b["pwm"], -500)
        self.assertEqual(res_mot_b["duration_ms"], 1200)



if __name__ == "__main__":
    unittest.main()

