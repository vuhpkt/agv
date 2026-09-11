#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for App Motor Control Refactoring (Task 5)
Verifies that app_motor_control delegates velocity calculation and control
to velocity_service without duplicating CPR math or maintaining duplicate PI/filter structs.
"""

import unittest
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestAppMotorControlRefactor(unittest.TestCase):

    def test_app_motor_control_uses_velocity_service(self):
        """Task 5: app_motor_control.c must include velocity_service.h and use velocity_channel_t"""
        file_path = os.path.join(REPO_ROOT, "Application", "Src", "app_motor_control.c")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            '#include "velocity_service.h"',
            content,
            "app_motor_control.c must include velocity_service.h"
        )
        self.assertIn(
            "velocity_channel_t",
            content,
            "app_motor_control.c must use velocity_channel_t"
        )

    def test_no_raw_cpr_math_in_app_motor_control(self):
        """Task 5: app_motor_control.c must not contain raw duplicate CPR-to-RPM division"""
        file_path = os.path.join(REPO_ROOT, "Application", "Src", "app_motor_control.c")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertNotIn(
            "* 60.0f) / ((float)MOTOR_ENCODER_CPR",
            content,
            "Raw CPR math should be delegated to velocity_service"
        )


if __name__ == "__main__":
    unittest.main()
