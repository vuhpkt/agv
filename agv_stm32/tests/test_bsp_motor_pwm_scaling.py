#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test: Verify BSP Motor PWM Duty Scaling
Ensures that BSP_Motor scales normalized duty [0, BSP_MOTOR_MAX_SPEED=1000]
to the hardware Timer period (TIM4 ARR = 3599).
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestBspMotorPwmScaling(unittest.TestCase):

    def test_bsp_motor_scales_duty_to_timer_period(self):
        """BSP_Motor must scale duty using __HAL_TIM_GET_AUTORELOAD or ARR, not raw duty directly into CCR"""
        bsp_motor_path = os.path.join(REPO_ROOT, "BSP", "Src", "bsp_motor.c")
        with open(bsp_motor_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Must not write raw duty directly: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty);
        self.assertNotIn(
            "__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty);",
            content,
            "bsp_motor.c should not write unscaled duty directly into CCR2"
        )
        self.assertNotIn(
            "__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);",
            content,
            "bsp_motor.c should not write unscaled duty directly into CCR3"
        )

        # Must scale using period and BSP_MOTOR_MAX_SPEED
        self.assertIn("__HAL_TIM_GET_AUTORELOAD", content)
        self.assertIn("BSP_MOTOR_MAX_SPEED", content)

    def test_scaling_math(self):
        """Math check: duty 400 with period 3599 and max 1000 must produce ~40% duty cycle (~1.96V on 4.9V Vm)"""
        max_speed = 1000
        period = 3599
        arr_counts = period + 1  # 3600

        duty_cmd = 400
        pulse = (duty_cmd * period) // max_speed
        duty_pct = pulse / arr_counts

        self.assertAlmostEqual(duty_pct, 0.40, delta=0.005)

        # Voltage check
        v_in = 4.9
        v_out = v_in * duty_pct
        self.assertAlmostEqual(v_out, 1.96, delta=0.03)

        # Compare with old unscaled bug:
        old_duty_pct = duty_cmd / arr_counts  # 400 / 3600 = 11.11%
        old_v_out = v_in * old_duty_pct
        self.assertAlmostEqual(old_v_out, 0.544, delta=0.01)  # Exact ~0.5V measured by user!


if __name__ == "__main__":
    unittest.main()

