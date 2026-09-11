#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for BSP IR Sensor Hardware Adapter (Task 1)
Verifies:
1. bsp_ir.h exists and defines BSP_IR_NUM_SENSORS == 5
2. Declarations for BSP_IR_Init and BSP_IR_GetRaw
3. bsp_ir.c implements calibration and DMA start
4. No direct extern access to internal DMA buffer (proper encapsulation)
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestBspIR(unittest.TestCase):

    def test_bsp_ir_header_exists_and_declares_api(self):
        """bsp_ir.h must define 5 sensors and declare Init and GetRaw"""
        header_path = os.path.join(REPO_ROOT, "BSP", "Inc", "bsp_ir.h")
        self.assertTrue(os.path.exists(header_path), f"bsp_ir.h does not exist at {header_path}")

        with open(header_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertRegex(
            content,
            r"#define\s+BSP_IR_NUM_SENSORS\s+5",
            "bsp_ir.h must define BSP_IR_NUM_SENSORS as 5"
        )
        self.assertRegex(
            content,
            r"void\s+BSP_IR_Init\s*\(\s*void\s*\)",
            "bsp_ir.h must declare void BSP_IR_Init(void)"
        )
        self.assertRegex(
            content,
            r"void\s+BSP_IR_GetRaw\s*\(\s*uint16_t\s+out_raw\s*\[\s*BSP_IR_NUM_SENSORS\s*\]\s*\)",
            "bsp_ir.h must declare void BSP_IR_GetRaw(uint16_t out_raw[BSP_IR_NUM_SENSORS])"
        )

    def test_bsp_ir_implementation_calls_calibration_and_dma(self):
        """bsp_ir.c must call HAL calibration and start DMA in circular mode"""
        src_path = os.path.join(REPO_ROOT, "BSP", "Src", "bsp_ir.c")
        self.assertTrue(os.path.exists(src_path), f"bsp_ir.c does not exist at {src_path}")

        with open(src_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            "HAL_ADCEx_Calibration_Start",
            content,
            "BSP_IR_Init must calibrate ADC before starting"
        )
        self.assertIn(
            "HAL_ADC_Start_DMA",
            content,
            "BSP_IR_Init must call HAL_ADC_Start_DMA"
        )
        self.assertRegex(
            content,
            r"static\s+(?:volatile\s+)?uint16_t\s+s_ir_raw\s*\[\s*BSP_IR_NUM_SENSORS\s*\]",
            "DMA buffer must be encapsulated as static uint16_t s_ir_raw[BSP_IR_NUM_SENSORS]"
        )

    def test_cmakelists_includes_bsp_ir(self):
        """CMakeLists.txt must compile BSP/Src/bsp_ir.c"""
        cmake_path = os.path.join(REPO_ROOT, "CMakeLists.txt")
        with open(cmake_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            "BSP/Src/bsp_ir.c",
            content,
            "CMakeLists.txt must include BSP/Src/bsp_ir.c in target_sources"
        )

    def test_app_init_calls_bsp_ir_init(self):
        """App_Init in Application/Src/app_main.c must initialize BSP_IR_Init"""
        app_main_path = os.path.join(REPO_ROOT, "Application", "Src", "app_main.c")
        with open(app_main_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            "BSP_IR_Init()",
            content,
            "Application/Src/app_main.c must call BSP_IR_Init() in App_Init"
        )


if __name__ == "__main__":
    unittest.main()
