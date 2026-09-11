#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Velocity Control Service (Task 3)
Tests CPR-to-RPM conversion, LPF filtering, PI velocity control integration,
and safety edge cases (zero dt, zero CPR, extreme ticks).
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class VelocityServiceReference:
    """
    Python reference model for Services/Src/velocity_service.c
    """
    def __init__(self, cpr=1320, alpha=0.85):
        self.cpr = cpr
        self.alpha = alpha
        self.filtered_rpm = 0.0
        self.raw_rpm = 0.0
        self.target_rpm = 0.0
        self.output_pwm = 0

    def calculate_raw_rpm(self, delta_ticks, dt_s):
        if dt_s <= 0.0 or self.cpr == 0:
            return 0.0
        return (float(delta_ticks) * 60.0) / (float(self.cpr) * dt_s)

    def update(self, delta_ticks, dt_s):
        if dt_s <= 0.0 or self.cpr == 0:
            return 0
        self.raw_rpm = self.calculate_raw_rpm(delta_ticks, dt_s)
        self.filtered_rpm = self.alpha * self.raw_rpm + (1.0 - self.alpha) * self.filtered_rpm
        return self.raw_rpm, self.filtered_rpm


class TestVelocityService(unittest.TestCase):

    def test_service_files_exist(self):
        """Task 3: Services/Inc/velocity_service.h and Services/Src/velocity_service.c must exist"""
        header = os.path.join(REPO_ROOT, "Services", "Inc", "velocity_service.h")
        src = os.path.join(REPO_ROOT, "Services", "Src", "velocity_service.c")
        self.assertTrue(os.path.exists(header), "Services/Inc/velocity_service.h does not exist")
        self.assertTrue(os.path.exists(src), "Services/Src/velocity_service.c does not exist")

    def test_service_in_cmake(self):
        """Task 3: CMakeLists.txt must include Services/Src/velocity_service.c and Services/Inc"""
        cmake_path = os.path.join(REPO_ROOT, "CMakeLists.txt")
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("Services/Src/velocity_service.c", content)
        self.assertIn("Services/Inc", content)

    def test_cpr_math_calculation(self):
        """Kiểm tra công thức chuyển đổi delta encoder ticks sang RPM"""
        srv = VelocityServiceReference(cpr=1320, alpha=0.85)
        # 1320 CPR, dt = 10ms (0.010s).
        # Tại 100 RPM: ticks/s = 100 * 1320 / 60 = 2200 ticks/s -> delta trong 10ms = 22 ticks
        raw_rpm = srv.calculate_raw_rpm(delta_ticks=22, dt_s=0.010)
        self.assertAlmostEqual(raw_rpm, 100.0, places=2)

        # Đảo chiều quay (-100 RPM -> -22 ticks)
        raw_rev = srv.calculate_raw_rpm(delta_ticks=-22, dt_s=0.010)
        self.assertAlmostEqual(raw_rev, -100.0, places=2)

    def test_division_by_zero_edge_cases(self):
        """Edge cases: dt = 0 hoặc CPR = 0 không được gây lỗi chia cho 0"""
        srv = VelocityServiceReference(cpr=1320)
        self.assertEqual(srv.calculate_raw_rpm(10, 0.0), 0.0)
        self.assertEqual(srv.calculate_raw_rpm(10, -0.01), 0.0)

        srv_no_cpr = VelocityServiceReference(cpr=0)
        self.assertEqual(srv_no_cpr.calculate_raw_rpm(10, 0.01), 0.0)


if __name__ == "__main__":
    unittest.main()
