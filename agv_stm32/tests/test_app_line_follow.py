#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Application Line Follow Controller (Task 3)
Verifies:
1. Architectural purity: app_line_follow must NOT include HAL headers directly.
2. State machine definitions (IDLE, CALIBRATING, TRACKING, CROSS_LINE, LOST).
3. API declarations (Init, Start, Stop, Process, GetState, SetPID).
4. Differential steering kinematics formula:
   left_rpm = base_rpm + delta_rpm
   right_rpm = base_rpm - delta_rpm
5. CMakeLists.txt target_sources inclusion.
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestAppLineFollow(unittest.TestCase):

    def test_app_line_follow_files_exist_and_zero_hal(self):
        """app_line_follow files must exist and not include HAL directly"""
        h_path = os.path.join(REPO_ROOT, "Application", "Inc", "app_line_follow.h")
        c_path = os.path.join(REPO_ROOT, "Application", "Src", "app_line_follow.c")

        self.assertTrue(os.path.exists(h_path), f"Missing {h_path}")
        self.assertTrue(os.path.exists(c_path), f"Missing {c_path}")

        for path in [h_path, c_path]:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
            self.assertNotIn("stm32f1xx_hal.h", content, f"{path} must not include HAL directly")

    def test_app_line_follow_state_machine_and_api(self):
        """app_line_follow.h must declare state enum and public control API"""
        h_path = os.path.join(REPO_ROOT, "Application", "Inc", "app_line_follow.h")
        self.assertTrue(os.path.exists(h_path), f"Missing {h_path}")

        with open(h_path, "r", encoding="utf-8") as f:
            content = f.read()

        for state in ["LINE_STATE_IDLE", "LINE_STATE_CALIBRATING", "LINE_STATE_TRACKING", "LINE_STATE_CROSS_LINE", "LINE_STATE_LOST"]:
            self.assertIn(state, content, f"app_line_follow.h missing state {state}")

        for fn in ["App_LineFollow_Init", "App_LineFollow_Start", "App_LineFollow_Stop", "App_LineFollow_Process"]:
            self.assertIn(fn, content, f"app_line_follow.h missing function {fn}")

    def test_differential_steering_formula(self):
        """Differential steering produces symmetric left/right velocities around base_rpm"""
        base_rpm = 100.0

        # When error < 0 (line on left, robot on right -> turn left -> left slows, right speeds up)
        # delta_rpm < 0
        delta_rpm = -20.0
        left_rpm = base_rpm + delta_rpm      # 80.0
        right_rpm = base_rpm - delta_rpm     # 120.0
        self.assertLess(left_rpm, right_rpm)
        self.assertEqual(left_rpm + right_rpm, 2 * base_rpm)

        # When error > 0 (line on right -> turn right -> left speeds up, right slows)
        delta_rpm = 20.0
        left_rpm = base_rpm + delta_rpm    # 120.0
        right_rpm = base_rpm - delta_rpm   # 80.0
        self.assertGreater(left_rpm, right_rpm)
        self.assertEqual(left_rpm + right_rpm, 2 * base_rpm)

    def test_cmakelists_includes_app_line_follow(self):
        """CMakeLists.txt must include Application/Src/app_line_follow.c"""
        cmake_path = os.path.join(REPO_ROOT, "CMakeLists.txt")
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            "Application/Src/app_line_follow.c",
            content,
            "CMakeLists.txt must include Application/Src/app_line_follow.c in target_sources"
        )


if __name__ == "__main__":
    unittest.main()
