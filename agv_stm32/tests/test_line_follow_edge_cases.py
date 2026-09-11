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


if __name__ == "__main__":
    unittest.main()
