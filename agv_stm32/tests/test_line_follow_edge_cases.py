import os
import re
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
APP_LINE_C = os.path.join(REPO_ROOT, "Application", "Src", "app_line_follow.c")
APP_LINE_H = os.path.join(REPO_ROOT, "Application", "Inc", "app_line_follow.h")


class TestLineFollowEdgeCases(unittest.TestCase):
    def test_line_follow_defines_timeout(self):
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("LINE_LOST_TIMEOUT_MS", content,
                      "app_line_follow.c must define LINE_LOST_TIMEOUT_MS")

        match = re.search(r"#define\s+LINE_LOST_TIMEOUT_MS\s+(\d+)", content)
        self.assertIsNotNone(match, "Could not parse LINE_LOST_TIMEOUT_MS value")
        timeout_val = int(match.group(1))
        self.assertGreaterEqual(timeout_val, 1000, "Timeout should be at least 1.0s")
        self.assertLessEqual(timeout_val, 5000, "Timeout should be at most 5.0s")

    def test_line_follow_handles_lost_line_safety_stop(self):
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("LINE_STATE_LOST", content)
        self.assertIn("App_MotorControl_Stop", content)
        self.assertIn("s_lost_start_tick", content,
                      "app_line_follow.c must track s_lost_start_tick for timeout safety")

    def test_line_follow_handles_crossline_stability(self):
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("LINE_STATE_CROSS_LINE", content)
        # Verify crossline does not steer wildly - check that is_crossline is handled
        self.assertIn("s_tracker.is_crossline", content)

    def test_crossline_debounce_is_200ms(self):
        """Verify CROSSLINE_DEBOUNCE_MS is defined as 200ms"""
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()

        match = re.search(r"#define\s+CROSSLINE_DEBOUNCE_MS\s+(\d+)", content)
        self.assertIsNotNone(match, "Could not parse CROSSLINE_DEBOUNCE_MS")
        debounce_ms = int(match.group(1))
        self.assertEqual(debounce_ms, 200, "CROSSLINE_DEBOUNCE_MS must be 200ms")

    def test_right_turn_masks_left_sensors_and_uses_pid(self):
        """Verify LINE_STATE_TURNING_RIGHT masks out sensors 3,4 and uses line_tracker_compute_pid"""
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("LINE_STATE_TURNING_RIGHT", content)
        self.assertIn("s_last_norm[3] = 0", content, "Right turn must zero sensor 3 (L1)")
        self.assertIn("s_last_norm[4] = 0", content, "Right turn must zero sensor 4 (L2)")
        self.assertIn("LINE_TRACKER_WEIGHT_OUTER", content, "Must use physical outer weight")
        self.assertIn("LINE_TRACKER_WEIGHT_INNER", content, "Must use physical inner weight")
        self.assertIn("line_tracker_compute_pid", content, "Must use Line PID for smooth turning")

    def test_right_turn_error_math_model(self):
        """Verify mathematical calculation of right-turn error with weights [1000, 385, 0]"""
        w_outer = 1000
        w_inner = 385
        w_center = 0

        # Scenario A: Line at outer sensor only (IR0=1000, others 0)
        norm_r2 = 1000
        norm_r1 = 0
        norm_mid = 0
        total = norm_r2 + norm_r1 + norm_mid
        err_a = int((norm_r2 * w_outer + norm_r1 * w_inner) / total)
        self.assertEqual(err_a, 1000, "Outer sensor only should give error +1000")

        # Scenario B: Line at inner sensor only (IR1=1000, others 0)
        norm_r2 = 0
        norm_r1 = 1000
        norm_mid = 0
        total = norm_r2 + norm_r1 + norm_mid
        err_b = int((norm_r2 * w_outer + norm_r1 * w_inner) / total)
        self.assertEqual(err_b, 385, "Inner sensor only should give error +385")

        # Scenario C: Line centered at mid sensor (IR2=1000, others 0)
        norm_r2 = 0
        norm_r1 = 0
        norm_mid = 1000
        total = norm_r2 + norm_r1 + norm_mid
        err_c = int((norm_r2 * w_outer + norm_r1 * w_inner) / total)
        self.assertEqual(err_c, 0, "Center sensor should give error 0")

        # Scenario D: Line between IR0 and IR1 (IR0=600, IR1=600, IR2=0)
        norm_r2 = 600
        norm_r1 = 600
        norm_mid = 0
        total = norm_r2 + norm_r1 + norm_mid
        err_d = int((norm_r2 * w_outer + norm_r1 * w_inner) / total)
        self.assertEqual(err_d, (1000 + 385) // 2)

    def test_crossing_for_turn_right_state_defined(self):
        """app_line_follow.h must define LINE_STATE_CROSSING_FOR_TURN_RIGHT"""
        with open(APP_LINE_H, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("LINE_STATE_CROSSING_FOR_TURN_RIGHT", content,
                      "app_line_follow.h must define LINE_STATE_CROSSING_FOR_TURN_RIGHT")

    def test_turn_max_timeout_defined(self):
        """app_line_follow.c must define TURN_MAX_TIMEOUT_MS as 3000ms"""
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()
        match = re.search(r"#define\s+TURN_MAX_TIMEOUT_MS\s+(\d+)", content)
        self.assertIsNotNone(match, "Could not find TURN_MAX_TIMEOUT_MS definition")
        self.assertEqual(int(match.group(1)), 3000, "TURN_MAX_TIMEOUT_MS must be 3000ms")

    def test_falling_edge_turn_right_logic(self):
        """app_line_follow.c must handle LINE_STATE_CROSSING_FOR_TURN_RIGHT and transition on !is_crossline"""
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("LINE_STATE_CROSSING_FOR_TURN_RIGHT", content)
        # When in crossing for turn right, must check !s_tracker.is_crossline to trigger LINE_STATE_TURNING_RIGHT
        pattern = r"LINE_STATE_CROSSING_FOR_TURN_RIGHT[\s\S]*?!s_tracker\.is_crossline[\s\S]*?LINE_STATE_TURNING_RIGHT"
        self.assertIsNotNone(re.search(pattern, content),
                             "Must transition from CROSSING_FOR_TURN_RIGHT to TURNING_RIGHT when is_crossline becomes false")

    def test_turn_right_timeout_protection(self):
        """LINE_STATE_TURNING_RIGHT must check TURN_MAX_TIMEOUT_MS and stop on timeout"""
        with open(APP_LINE_C, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("TURN_MAX_TIMEOUT_MS", content)
        pattern = r"LINE_STATE_TURNING_RIGHT[\s\S]*?TURN_MAX_TIMEOUT_MS[\s\S]*?App_LineFollow_Stop"
        self.assertIsNotNone(re.search(pattern, content),
                             "LINE_STATE_TURNING_RIGHT must trigger App_LineFollow_Stop on TURN_MAX_TIMEOUT_MS")


if __name__ == "__main__":
    unittest.main()
