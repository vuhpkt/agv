#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Diagnostics Motor Module (Task 1)
Verifies that diag_motor is located in Diagnostics/, provides Diag_Motor_* APIs,
delegates velocity calculation to velocity_service, and old app_step_test is removed.
"""

import unittest
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestDiagMotor(unittest.TestCase):

    def test_diag_motor_header_exists(self):
        """diag_motor.h must exist in Diagnostics/Inc"""
        header_path = os.path.join(REPO_ROOT, "Diagnostics", "Inc", "diag_motor.h")
        self.assertTrue(os.path.exists(header_path), "Diagnostics/Inc/diag_motor.h must exist")
        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()

        expected_symbols = [
            "DIAG_MOTOR_H",
            "Diag_Motor_Init",
            "Diag_Motor_Start",
            "Diag_Motor_StartDefault",
            "Diag_Motor_Stop",
            "Diag_Motor_IsRunning",
            "Diag_Motor_GetLastResult",
            "Diag_Motor_Process",
            "Diag_Motor_Status_t",
            "Diag_Motor_Result_t",
        ]
        for sym in expected_symbols:
            self.assertIn(sym, content, f"diag_motor.h must declare {sym}")

    def test_diag_motor_source_exists(self):
        """diag_motor.c must exist in Diagnostics/Src and include velocity_service.h"""
        source_path = os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_motor.c")
        self.assertTrue(os.path.exists(source_path), "Diagnostics/Src/diag_motor.c must exist")
        with open(source_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn('#include "diag_motor.h"', content)
        self.assertIn('#include "velocity_service.h"', content)
        self.assertNotIn(
            "* 60.0f) / ((float)STEP_TEST_CPR",
            content,
            "Raw duplicate CPR math should be delegated to velocity_service"
        )

    def test_old_app_step_test_removed(self):
        """Old app_step_test files in Application/ must be deleted"""
        old_h = os.path.join(REPO_ROOT, "Application", "Inc", "app_step_test.h")
        old_c = os.path.join(REPO_ROOT, "Application", "Src", "app_step_test.c")
        self.assertFalse(os.path.exists(old_h), "Application/Inc/app_step_test.h must be removed")
        self.assertFalse(os.path.exists(old_c), "Application/Src/app_step_test.c must be removed")

    def test_direction_check_logic_in_diag_motor(self):
        """diag_motor.c must implement DIR_CHECK polarity verification between PWM and Ticks"""
        source_path = os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_motor.c")
        with open(source_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("DIR_CHECK", content, "diag_motor.c must output [DIR_CHECK] log")
        self.assertIn("labs(", content, "diag_motor.c should use labs for 32-bit total_encoder_ticks")



if __name__ == "__main__":
    unittest.main()
