#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Suite for Architecture Boundaries & Time Abstraction (Task 1 & Task 2)
Verifies clean layering rules:
1. Application layer must NOT call HAL_GetTick() or include HAL headers.
2. BSP Board must expose bsp_time_get_ms() and bsp_time_delay_ms().
3. Unsigned 32-bit tick rollover math behaves correctly across overflows.
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestArchitectureBoundaries(unittest.TestCase):

    def test_bsp_board_exposes_time_api(self):
        """Task 1: bsp_board.h must declare bsp_time_get_ms and bsp_time_delay_ms"""
        header_path = os.path.join(REPO_ROOT, "BSP", "Inc", "bsp_board.h")
        self.assertTrue(os.path.exists(header_path), "bsp_board.h does not exist")

        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertRegex(
            content,
            r"uint32_t\s+bsp_time_get_ms\s*\(\s*void\s*\)",
            "bsp_board.h must declare uint32_t bsp_time_get_ms(void)"
        )
        self.assertRegex(
            content,
            r"void\s+bsp_time_delay_ms\s*\(\s*uint32_t",
            "bsp_board.h must declare void bsp_time_delay_ms(uint32_t ms)"
        )

    def test_no_hal_get_tick_in_application_layer(self):
        """Task 1: Application layer must not call HAL_GetTick() directly"""
        app_dir = os.path.join(REPO_ROOT, "Application")
        violations = []

        for root, _, files in os.walk(app_dir):
            for file in files:
                if file.endswith((".c", ".h")):
                    path = os.path.join(root, file)
                    with open(path, "r", encoding="utf-8") as f:
                        for line_num, line in enumerate(f, start=1):
                            if "HAL_GetTick" in line:
                                rel_path = os.path.relpath(path, REPO_ROOT)
                                violations.append(f"{rel_path}:{line_num}: {line.strip()}")

        self.assertEqual(
            violations,
            [],
            f"Found direct HAL_GetTick calls in Application layer:\n" + "\n".join(violations)
        )

    def test_uint32_tick_overflow_edge_cases(self):
        """
        Verify the non-blocking timer interval calculation handles 32-bit tick rollover:
        (uint32_t)(current_tick - last_tick) >= interval
        """
        UINT32_MAX = 0xFFFFFFFF
        interval = 10  # 10ms loop

        # Normal case
        last_tick = 1000
        current_tick = 1010
        elapsed = (current_tick - last_tick) & UINT32_MAX
        self.assertGreaterEqual(elapsed, interval)

        # Edge case: Rollover near 2^32 - 1
        last_tick = UINT32_MAX - 5   # 5 ticks before rollover
        current_tick = 5             # 5 ticks after rollover
        elapsed = (current_tick - last_tick) & UINT32_MAX
        # (5 - (2^32 - 6)) mod 2^32 = 11 ticks elapsed
        self.assertEqual(elapsed, 11)
        self.assertGreaterEqual(elapsed, interval)

        # Edge case: current == last
        last_tick = 500
        current_tick = 500
        elapsed = (current_tick - last_tick) & UINT32_MAX
        self.assertLess(elapsed, interval)

    def test_stm32f1xx_it_does_not_include_bsp_uart(self):
        """Task 2: Core/Src/stm32f1xx_it.c must not include bsp_uart.h directly"""
        it_c_path = os.path.join(REPO_ROOT, "Core", "Src", "stm32f1xx_it.c")
        with open(it_c_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertNotIn(
            '#include "bsp_uart.h"',
            content,
            "stm32f1xx_it.c should not tightly couple to bsp_uart.h"
        )

    def test_uart_callbacks_in_bsp_uart_not_in_it(self):
        """Task 2: HAL_UART callbacks must be defined in bsp_uart.c, not in stm32f1xx_it.c"""
        it_c_path = os.path.join(REPO_ROOT, "Core", "Src", "stm32f1xx_it.c")
        with open(it_c_path, "r", encoding="utf-8") as f:
            it_content = f.read()

        bsp_uart_c_path = os.path.join(REPO_ROOT, "BSP", "Src", "bsp_uart.c")
        with open(bsp_uart_c_path, "r", encoding="utf-8") as f:
            bsp_content = f.read()

        self.assertNotIn(
            "void HAL_UART_TxCpltCallback",
            it_content,
            "HAL_UART_TxCpltCallback should be moved out of stm32f1xx_it.c"
        )
        self.assertNotIn(
            "void HAL_UART_ErrorCallback",
            it_content,
            "HAL_UART_ErrorCallback should be moved out of stm32f1xx_it.c"
        )
        self.assertIn(
            "void HAL_UART_TxCpltCallback",
            bsp_content,
            "bsp_uart.c must implement HAL_UART_TxCpltCallback"
        )
        self.assertIn(
            "void HAL_UART_ErrorCallback",
            bsp_content,
            "bsp_uart.c must implement HAL_UART_ErrorCallback"
        )

    def test_components_directory_removed(self):
        """Components directory must be completely removed"""
        components_dir = os.path.join(REPO_ROOT, "Components")
        self.assertFalse(
            os.path.exists(components_dir),
            f"Components directory should not exist: {components_dir}"
        )

    def test_drivers_directory_structure(self):
        """Drivers must host tb6612 IC driver and SEGGER_RTT debug driver"""
        expected_files = [
            os.path.join(REPO_ROOT, "Drivers", "tb6612", "tb6612.h"),
            os.path.join(REPO_ROOT, "Drivers", "tb6612", "tb6612.c"),
            os.path.join(REPO_ROOT, "Drivers", "SEGGER_RTT", "SEGGER_RTT.h"),
            os.path.join(REPO_ROOT, "Drivers", "SEGGER_RTT", "SEGGER_RTT.c"),
            os.path.join(REPO_ROOT, "Drivers", "SEGGER_RTT", "SEGGER_RTT_printf.c"),
        ]
        for path in expected_files:
            self.assertTrue(os.path.exists(path), f"Missing driver file: {path}")

    def test_services_directory_structure(self):
        """Services/Inc and Services/Src must host all pure algorithms and business services"""
        headers = [
            "comms_protocol.h",
            "velocity_service.h",
            "encoder.h",
            "filter.h",
            "kinematics.h",
            "motor_pi.h",
            "pid.h",
            "ring_buffer.h",
        ]
        sources = [
            "comms_protocol.c",
            "velocity_service.c",
            "encoder.c",
            "filter.c",
            "kinematics.c",
            "motor_pi.c",
            "pid.c",
            "ring_buffer.c",
        ]
        for h in headers:
            path = os.path.join(REPO_ROOT, "Services", "Inc", h)
            self.assertTrue(os.path.exists(path), f"Missing Service header: {path}")

        for s in sources:
            path = os.path.join(REPO_ROOT, "Services", "Src", s)
            self.assertTrue(os.path.exists(path), f"Missing Service source: {path}")

    def test_diagnostics_directory_structure(self):
        """Diagnostics/Inc and Diagnostics/Src must host diagnostic hardware modules"""
        expected_files = [
            os.path.join(REPO_ROOT, "Diagnostics", "Inc", "diag_motor.h"),
            os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_motor.c"),
            os.path.join(REPO_ROOT, "Diagnostics", "Inc", "diag_ir.h"),
            os.path.join(REPO_ROOT, "Diagnostics", "Src", "diag_ir.c"),
        ]
        for path in expected_files:
            self.assertTrue(os.path.exists(path), f"Missing Diagnostics file: {path}")

    def test_no_test_code_in_application_layer(self):
        """Application/ directory must be clean of test modules (no app_step_test or app_line_test)"""
        app_dir = os.path.join(REPO_ROOT, "Application")
        for root, _, files in os.walk(app_dir):
            for file in files:
                lower = file.lower()
                self.assertNotIn("test", lower, f"Test module found in Application: {file}")


if __name__ == "__main__":
    unittest.main()

