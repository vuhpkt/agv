import os
import re
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HEADER_PATH = os.path.join(REPO_ROOT, "Application", "Inc", "app_line_calib_defaults.h")
SOURCE_PATH = os.path.join(REPO_ROOT, "Application", "Src", "app_line_follow.c")


class TestLineCalibDefaults(unittest.TestCase):
    def test_header_file_exists(self):
        self.assertTrue(os.path.isfile(HEADER_PATH), f"Missing header: {HEADER_PATH}")

    def test_header_defines_valid_arrays(self):
        with open(HEADER_PATH, "r", encoding="utf-8") as f:
            content = f.read()

        min_match = re.search(r"#define\s+APP_LINE_CALIB_MIN_RAW\s*\{([^}]+)\}", content)
        max_match = re.search(r"#define\s+APP_LINE_CALIB_MAX_RAW\s*\{([^}]+)\}", content)

        self.assertIsNotNone(min_match, "APP_LINE_CALIB_MIN_RAW macro not found in header")
        self.assertIsNotNone(max_match, "APP_LINE_CALIB_MAX_RAW macro not found in header")

        min_vals = [int(x.strip()) for x in min_match.group(1).split(",") if x.strip()]
        max_vals = [int(x.strip()) for x in max_match.group(1).split(",") if x.strip()]

        self.assertEqual(len(min_vals), 5, f"Expected 5 min values, got {len(min_vals)}")
        self.assertEqual(len(max_vals), 5, f"Expected 5 max values, got {len(max_vals)}")

        for idx, (min_v, max_v) in enumerate(zip(min_vals, max_vals)):
            self.assertGreaterEqual(min_v, 0, f"Sensor {idx} min value < 0: {min_v}")
            self.assertLessEqual(max_v, 4095, f"Sensor {idx} max value > 4095: {max_v}")
            self.assertLess(min_v, max_v, f"Sensor {idx} min >= max ({min_v} >= {max_v})")
            self.assertGreaterEqual(
                max_v - min_v, 100, f"Sensor {idx} dynamic range too low: {max_v - min_v} < 100"
            )

    def test_app_line_follow_includes_header_and_uses_defaults(self):
        with open(SOURCE_PATH, "r", encoding="utf-8") as f:
            source = f.read()

        self.assertIn('#include "app_line_calib_defaults.h"', source,
                      "app_line_follow.c must include app_line_calib_defaults.h")
        self.assertIn("APP_LINE_CALIB_MIN_RAW", source,
                      "app_line_follow.c must use APP_LINE_CALIB_MIN_RAW")
        self.assertIn("APP_LINE_CALIB_MAX_RAW", source,
                      "app_line_follow.c must use APP_LINE_CALIB_MAX_RAW")


if __name__ == "__main__":
    unittest.main()
