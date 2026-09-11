#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for scripts/test_motor_direction.py (Task 3)
Tests diagnostic logic:
- Polarity matching
- Recommendation generation for CubeMX / wiring fixes
- Telemetry line parsing from RTT
"""

import unittest
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from test_motor_direction import (
    diagnose_result,
    parse_rtt_step_response,
    DIAG_ALL_OK,
    DIAG_ENCODER_INVERTED,
    DIAG_MOTOR_INVERTED,
    DIAG_BOTH_INVERTED,
    DIAG_NO_MOTION,
)


class TestMotorDirectionScript(unittest.TestCase):

    def test_script_exists(self):
        """scripts/test_motor_direction.py must exist"""
        script_path = os.path.join(REPO_ROOT, "scripts", "test_motor_direction.py")
        self.assertTrue(os.path.exists(script_path))

    def test_diagnose_all_ok(self):
        """Khi bánh xe quay đúng chiều và encoder đếm đúng chiều -> DIAG_ALL_OK"""
        # Forward test: PWM > 0, wheel spins FWD, ticks > 0
        diag_fwd = diagnose_result(pwm=400, wheel_spins_fwd=True, ticks=1250)
        self.assertEqual(diag_fwd["status"], DIAG_ALL_OK)
        self.assertTrue(diag_fwd["wheel_ok"])
        self.assertTrue(diag_fwd["encoder_ok"])

        # Reverse test: PWM < 0, wheel spins REV, ticks < 0
        diag_rev = diagnose_result(pwm=-400, wheel_spins_fwd=False, ticks=-1250)
        self.assertEqual(diag_rev["status"], DIAG_ALL_OK)
        self.assertTrue(diag_rev["wheel_ok"])
        self.assertTrue(diag_rev["encoder_ok"])

    def test_diagnose_encoder_inverted(self):
        """Khi bánh xe quay đúng chiều nhưng encoder đếm ngược -> DIAG_ENCODER_INVERTED"""
        # PWM +400, wheel spins FWD, but ticks is negative (-1200)
        diag = diagnose_result(pwm=400, wheel_spins_fwd=True, ticks=-1200)
        self.assertEqual(diag["status"], DIAG_ENCODER_INVERTED)
        self.assertTrue(diag["wheel_ok"])
        self.assertFalse(diag["encoder_ok"])
        self.assertIn("Phase A/B", diag["recommendation"])

    def test_diagnose_motor_inverted(self):
        """Khi bánh xe quay ngược chiều lệnh PWM và encoder đếm theo chiều bánh -> DIAG_MOTOR_INVERTED"""
        # Lệnh PWM +400 (Tiến), nhưng bánh xe quay Lùi (wheel_spins_fwd=False), ticks đếm lùi (-1200)
        diag = diagnose_result(pwm=400, wheel_spins_fwd=False, ticks=-1200)
        self.assertEqual(diag["status"], DIAG_MOTOR_INVERTED)
        self.assertFalse(diag["wheel_ok"])
        self.assertIn("CubeMX", diag["recommendation"])

    def test_diagnose_both_inverted(self):
        """Khi bánh xe quay ngược nhưng encoder lại đếm dương -> DIAG_BOTH_INVERTED"""
        # Lệnh PWM +400 (Tiến), bánh quay Lùi (False), nhưng ticks lại dương (+1200)
        diag = diagnose_result(pwm=400, wheel_spins_fwd=False, ticks=1200)
        self.assertEqual(diag["status"], DIAG_BOTH_INVERTED)
        self.assertFalse(diag["wheel_ok"])
        self.assertFalse(diag["encoder_ok"])

    def test_diagnose_no_motion(self):
        """Khi không phát hiện chuyển động xung -> DIAG_NO_MOTION"""
        diag = diagnose_result(pwm=400, wheel_spins_fwd=True, ticks=5)
        self.assertEqual(diag["status"], DIAG_NO_MOTION)

    def test_parse_rtt_step_response(self):
        """Trích xuất chính xác Ticks, Peak RPM, và PASS/FAIL từ log RTT"""
        sample_log = (
            "DATA_START\r\n"
            "Time_ms,Time_s,PWM,PWM_Percent,DeltaTicks,Speed_RPM,Speed_RadS\r\n"
            "DATA:100,0.1000,400,40.0,15,68.20,7.142\r\n"
            "DATA_END\r\n"
            "[STEP_TEST] TEST_PASS | Peak: 72.5 RPM | Steady: 70.1 RPM | Ticks: 1342 | Samples: 200\r\n"
            "[DIR_CHECK] PASS: Motor PWM (400) & Encoder Ticks (1342) MATCH (+Forward/+Reverse)\r\n"
        )
        parsed = parse_rtt_step_response(sample_log)
        self.assertEqual(parsed["ticks"], 1342)
        self.assertAlmostEqual(parsed["peak_rpm"], 72.5)
        self.assertTrue(parsed["passed"])


if __name__ == "__main__":
    unittest.main()
