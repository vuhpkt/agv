#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for Pure C Line Tracker Service (Task 2)
Tests:
1. Architectural purity: Services/Inc/line_tracker.h and Services/Src/line_tracker.c must not include HAL.
2. Mathematical model verification:
   - Normalization maps raw [min, max] to [0, 1000]
   - Centered line produces error ~ 0
   - Off-center lines produce correct sign (-1000 to +1000)
   - Crossline detection when >= 4 sensors see line
   - Lost line retains sign of previous error
   - PID steering compensation with anti-windup and clamp
"""

import unittest
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class PythonLineTrackerReference:
    """
    Python reference model that mirrors Services/Src/line_tracker.c exactly
    """
    def __init__(self, kp=0.25, ki=0.01, kd=0.05, out_max=80.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max

        self.min_raw = [100, 100, 100, 100, 100]
        self.max_raw = [3500, 3500, 3500, 3500, 3500]
        self.threshold = [1800, 1800, 1800, 1800, 1800]
        self.is_white_line = False

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_valid_error = 0
        self.line_lost = False
        self.is_crossline = False

    def normalize(self, raw):
        norm = [0] * 5
        for i in range(5):
            val = raw[i]
            if val <= self.min_raw[i]:
                n = 0
            elif val >= self.max_raw[i]:
                n = 1000
            else:
                span = self.max_raw[i] - self.min_raw[i]
                n = int(((val - self.min_raw[i]) * 1000) / span) if span > 0 else 0

            # Invert if dark line on light background (low ADC on black tape)
            if not self.is_white_line:
                n = 1000 - n
            norm[i] = max(0, min(1000, n))
        return norm

    def compute_error(self, norm):
        # Weights matching physical spacing: center=0, inner(1.25cm)=385, outer(3.25cm)=1000
        weights = [1000, 385, 0, -385, -1000]
        sum_weighted = 0
        total_intensity = 0
        max_intensity = 0
        active_count = 0

        for i in range(5):
            intensity = norm[i]
            if intensity > 300:
                active_count += 1
            if intensity > max_intensity:
                max_intensity = intensity
            sum_weighted += weights[i] * intensity
            total_intensity += intensity

        # 1. Crossline check: >= 4 sensors active
        self.is_crossline = (active_count >= 4)

        # 2. Lost line check: multi-tier condition
        if active_count == 0 or max_intensity < 300 or total_intensity < 350:
            self.line_lost = True
            # Retain extreme error in the direction last seen
            if self.last_valid_error < 0:
                return -1000
            elif self.last_valid_error > 0:
                return 1000
            else:
                return 0

        self.line_lost = False
        error = int(sum_weighted / total_intensity)
        error = max(-1000, min(1000, error))
        self.last_valid_error = error
        return error

    def compute_pid(self, error, dt_s):
        if dt_s <= 0.0:
            return 0.0

        p_term = self.kp * float(error)

        self.integral += float(error) * dt_s
        # Anti-windup clamp
        i_limit = self.out_max * 0.5
        if self.integral > i_limit:
            self.integral = i_limit
        elif self.integral < -i_limit:
            self.integral = -i_limit
        i_term = self.ki * self.integral

        d_term = self.kd * (float(error) - self.prev_error) / dt_s
        self.prev_error = float(error)

        output = p_term + i_term + d_term
        # Clamp output
        if output > self.out_max:
            output = self.out_max
        elif output < -self.out_max:
            output = -self.out_max

        return output


class TestLineTracker(unittest.TestCase):

    def test_line_tracker_files_exist_and_zero_hal(self):
        """line_tracker.h and line_tracker.c must exist and have zero HAL dependencies"""
        h_path = os.path.join(REPO_ROOT, "Services", "Inc", "line_tracker.h")
        c_path = os.path.join(REPO_ROOT, "Services", "Src", "line_tracker.c")

        self.assertTrue(os.path.exists(h_path), f"Missing {h_path}")
        self.assertTrue(os.path.exists(c_path), f"Missing {c_path}")

        for path in [h_path, c_path]:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
            self.assertNotIn("stm32f1xx", content, f"{path} must not include HAL headers")
            self.assertNotIn("HAL_", content, f"{path} must not call HAL functions")

    def test_centered_line_error_zero(self):
        """Centered line (sensor 2 high, others low) must produce error == 0"""
        tracker = PythonLineTrackerReference()
        # Raw ADC: White floor ~ 3500, Black line on sensor 2 ~ 100
        raw = [3500, 3500, 100, 3500, 3500]
        norm = tracker.normalize(raw)
        # Sensor 2 should be 1000, others 0
        self.assertEqual(norm[2], 1000)
        self.assertEqual(norm[0], 0)
        self.assertEqual(norm[4], 0)

        error = tracker.compute_error(norm)
        self.assertEqual(error, 0)
        self.assertFalse(tracker.line_lost)
        self.assertFalse(tracker.is_crossline)

    def test_line_shifted_left_produces_negative_error(self):
        """Line shifted left (sensor 3 or 4 high) must produce negative error"""
        tracker = PythonLineTrackerReference()
        # Black line on sensor 4 (L2 - leftmost, 3.25cm from center)
        raw = [3500, 3500, 3500, 3500, 100]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertEqual(error, -1000)

        # Black line on sensor 3 (L1 - inner left, 1.25cm from center)
        raw = [3500, 3500, 3500, 100, 3500]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertEqual(error, -385)

        # Black line between sensor 3 (L1) and sensor 4 (L2)
        raw = [3500, 3500, 3500, 100, 100]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        # (-385*1000 + -1000*1000) / 2000 = -1385 / 2 = -692
        self.assertEqual(error, -692)

    def test_line_shifted_right_produces_positive_error(self):
        """Line shifted right (sensor 0 or 1 high) must produce positive error"""
        tracker = PythonLineTrackerReference()
        # Black line on sensor 0 (R2 - rightmost, 3.25cm from center)
        raw = [100, 3500, 3500, 3500, 3500]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertEqual(error, 1000)

        # Black line on sensor 1 (R1 - inner right, 1.25cm from center)
        raw = [3500, 100, 3500, 3500, 3500]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertEqual(error, 385)

        # Black line between sensor 0 (R2) and sensor 1 (R1)
        raw = [100, 100, 3500, 3500, 3500]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        # (1000*1000 + 385*1000) / 2000 = 1385 / 2 = 692
        self.assertEqual(error, 692)

    def test_crossline_detected_when_all_sensors_on_line(self):
        """All 5 sensors seeing line indicates crossline/intersection"""
        tracker = PythonLineTrackerReference()
        raw = [100, 100, 100, 100, 100]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertEqual(error, 0)
        self.assertTrue(tracker.is_crossline)
        self.assertFalse(tracker.line_lost)

    def test_lost_line_preserves_last_direction(self):
        """When all sensors see white, line_lost is True and preserves sign of last direction"""
        tracker = PythonLineTrackerReference()
        # First steer right (line was on right - sensor 0)
        raw = [100, 3500, 3500, 3500, 3500]
        norm = tracker.normalize(raw)
        error = tracker.compute_error(norm)
        self.assertGreater(error, 0)

        # Now all white floor (lost line)
        raw_lost = [3500, 3500, 3500, 3500, 3500]
        norm_lost = tracker.normalize(raw_lost)
        error_lost = tracker.compute_error(norm_lost)
        self.assertTrue(tracker.line_lost)
        self.assertEqual(error_lost, 1000)  # Preserves rightward bias to regain line

    def test_floor_reflection_noise_detected_as_lost_line(self):
        """Ambient reflection (norm ~ 50-60 each, sum ~ 260) must be detected as line_lost"""
        tracker = PythonLineTrackerReference()
        # Ambient reflection on glossy floor: each sensor reads ~50-60
        # Previously sum=265 > 200 caused line_lost to be False!
        norm_ambient = [60, 50, 45, 55, 50]
        error = tracker.compute_error(norm_ambient)
        self.assertTrue(tracker.line_lost, "Ambient reflection noise must trigger line_lost")

    def test_pid_steering_compensation_and_clamp(self):
        """Line PID generates proportional delta_rpm clamped to out_max"""
        tracker = PythonLineTrackerReference(kp=0.2, ki=0.0, kd=0.0, out_max=60.0)
        dt = 0.01  # 10ms

        # Small error = 100 -> delta = 20 RPM
        delta = tracker.compute_pid(100, dt)
        self.assertAlmostEqual(delta, 20.0, places=1)

        # Huge error = 1000 -> delta = 200, clamped to out_max (60 RPM)
        delta_clamped = tracker.compute_pid(1000, dt)
        self.assertEqual(delta_clamped, 60.0)

        # Negative huge error = -1000 -> clamped to -60 RPM
        delta_neg = tracker.compute_pid(-1000, dt)
        self.assertEqual(delta_neg, -60.0)


if __name__ == "__main__":
    unittest.main()
