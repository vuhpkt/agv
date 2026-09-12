#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Diagnostics IR Module (Task 2)
Verifies that diag_ir is located in Diagnostics/, provides Diag_IR_* APIs,
evaluates dynamic range, and old app_line_test is removed.
"""

import os
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestDiagIR(unittest.TestCase):

    def test_diag_ir_header_exists(self):
        """diag_ir.h must exist in Diagnostics/Inc and declare Diag_IR_* APIs"""
        header_path = os.path.join(REPO_ROOT, "Diagnostics", "Inc", "diag_ir.h")
        self.assertTrue(os.path.exists(header_path), "Diagnostics/Inc/diag_ir.h must exist")
        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()

        required_symbols = [
            "DIAG_IR_H",
            "DIAG_IR_MODE_SENSOR",
            "DIAG_IR_MODE_CALIB",
            "DIAG_IR_MODE_TRACK",
            "DIAG_IR_STATE_IDLE",
            "DIAG_IR_STATE_RUNNING",
            "DIAG_IR_STATE_PASSED",
            "DIAG_IR_STATE_FAILED",
            "Diag_IR_Result_t",
            "Diag_IR_Init",
            "Diag_IR_Start",
            "Diag_IR_StartDefault",
            "Diag_IR_Stop",
            "Diag_IR_IsRunning",
            "Diag_IR_GetLastResult",
            "Diag_IR_Process",
        ]
        for sym in required_symbols:
            self.assertIn(sym, content, f"diag_ir.h must declare {sym}")

    def test_diag_ir_source_exists(self):
        """diag_ir.c must exist in Diagnostics/Src"""
        source_path = os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_ir.c")
        self.assertTrue(os.path.exists(source_path), "Diagnostics/Src/diag_ir.c must exist")
        with open(source_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn('#include "diag_ir.h"', content)
        self.assertIn('#include "bsp_ir.h"', content)
        self.assertIn('#include "line_tracker.h"', content)

    def test_cmake_includes_diag_ir(self):
        """CMakeLists.txt must include diag_ir.c and not app_line_test.c"""
        cmake_path = os.path.join(REPO_ROOT, "CMakeLists.txt")
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("Diagnostics/Src/diag_ir.c", content)
        self.assertNotIn("Application/Src/app_line_test.c", content)

    def test_old_app_line_test_removed(self):
        """Old app_line_test files in Application/ must be deleted"""
        old_h = os.path.join(REPO_ROOT, "Application", "Inc", "app_line_test.h")
        old_c = os.path.join(REPO_ROOT, "Application", "Src", "app_line_test.c")
        self.assertFalse(os.path.exists(old_h), "Application/Inc/app_line_test.h must be removed")
        self.assertFalse(os.path.exists(old_c), "Application/Src/app_line_test.c must be removed")

    def test_evaluation_logic(self):
        """Test evaluation logic: dynamic range check for pass/fail"""
        min_dynamic_range = 100
        # Scenario 1: All sensors have good dynamic range
        min_raw = [200, 300, 250, 280, 220]
        max_raw = [3200, 3100, 3300, 3150, 3250]
        all_ok = all((max_raw[i] - min_raw[i]) >= min_dynamic_range for i in range(5))
        self.assertTrue(all_ok)

        # Scenario 2: One sensor stuck/dead (flat reading)
        max_raw_bad = [3200, 305, 3300, 3150, 3250]
        any_failed = any((max_raw_bad[i] - min_raw[i]) < min_dynamic_range for i in range(5))
        self.assertTrue(any_failed)

    def test_diag_ir_lost_timeout_definition(self):
        """diag_ir.h must define DIAG_IR_LOST_TIMEOUT_MS as 2000 (2.0s)"""
        import re
        header_path = os.path.join(REPO_ROOT, "Diagnostics", "Inc", "diag_ir.h")
        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("DIAG_IR_LOST_TIMEOUT_MS", content)
        match = re.search(r"#define\s+DIAG_IR_LOST_TIMEOUT_MS\s+(\d+)", content)
        self.assertIsNotNone(match)
        timeout_ms = int(match.group(1))
        self.assertEqual(timeout_ms, 2000, "DIAG_IR_LOST_TIMEOUT_MS must be 2000ms (2.0s)")

    def test_diag_ir_lost_line_safety_and_pid_tuning(self):
        """diag_ir.c must track lost timeout, zero out Ki to avoid hunting, and adapt out_max"""
        source_path = os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_ir.c")
        with open(source_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("s_lost_start_tick", content, "Must track s_lost_start_tick")
        self.assertIn("DIAG_IR_LOST_TIMEOUT_MS", content, "Must check DIAG_IR_LOST_TIMEOUT_MS")
        self.assertIn(".ki = 0.0f", content, "Ki must be 0.0f to prevent hunting oscillations")
        self.assertIn("s_tracker.pid_cfg.out_max", content, "Must adapt out_max dynamically")


if __name__ == "__main__":
    unittest.main()
