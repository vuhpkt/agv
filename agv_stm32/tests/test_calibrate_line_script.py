import json
import os
import tempfile
import unittest

# Import the module to be implemented
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
import sys

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from scripts.calibrate_line import (
    CalibrationStats,
    parse_line_telemetry,
    validate_calibration,
    generate_header_content,
    save_calib_json,
    load_calib_json,
    write_calib_header,
)


class TestCalibrateLineScript(unittest.TestCase):
    def test_parse_line_telemetry_valid(self):
        line = "DATA:LINE,150,450,520,3100,510,480,10,12,980,11,10,0,1"
        raw = parse_line_telemetry(line)
        self.assertIsNotNone(raw)
        self.assertEqual(raw, [450, 520, 3100, 510, 480])

    def test_parse_line_telemetry_invalid(self):
        self.assertIsNone(parse_line_telemetry("ACK:OK"))
        self.assertIsNone(parse_line_telemetry("DATA:STEP,100,20.0,21.0"))
        self.assertIsNone(parse_line_telemetry(""))

    def test_calibration_stats_update(self):
        stats = CalibrationStats(num_sensors=5)
        self.assertEqual(stats.sample_count, 0)

        stats.update([500, 400, 600, 450, 550])
        stats.update([200, 3500, 300, 3200, 250])
        stats.update([3400, 300, 3600, 400, 3300])

        self.assertEqual(stats.sample_count, 3)
        self.assertEqual(stats.min_raw, [200, 300, 300, 400, 250])
        self.assertEqual(stats.max_raw, [3400, 3500, 3600, 3200, 3300])

    def test_validate_calibration_pass(self):
        min_raw = [300, 320, 310, 305, 330]
        max_raw = [3400, 3500, 3450, 3420, 3500]
        is_valid, errors = validate_calibration(min_raw, max_raw, min_dynamic_range=100)
        self.assertTrue(is_valid)
        self.assertEqual(len(errors), 0)

    def test_validate_calibration_fail_low_dynamic_range(self):
        min_raw = [300, 320, 310, 305, 330]
        max_raw = [3400, 350, 3450, 3420, 3500]  # Sensor 1 delta = 30 < 100
        is_valid, errors = validate_calibration(min_raw, max_raw, min_dynamic_range=100)
        self.assertFalse(is_valid)
        self.assertEqual(len(errors), 1)
        self.assertTrue("Ch1" in errors[0] or "Sensor 1" in errors[0])

    def test_generate_header_content(self):
        min_raw = [300, 310, 320, 330, 340]
        max_raw = [3400, 3410, 3420, 3430, 3440]
        content = generate_header_content(min_raw, max_raw)

        self.assertIn("#ifndef APP_LINE_CALIB_DEFAULTS_H", content)
        self.assertIn("#define APP_LINE_CALIB_MIN_RAW   { 300, 310, 320, 330, 340 }", content)
        self.assertIn("#define APP_LINE_CALIB_MAX_RAW   { 3400, 3410, 3420, 3430, 3440 }", content)
        self.assertIn("#endif", content)

    def test_save_and_load_json(self):
        min_raw = [300, 310, 320, 330, 340]
        max_raw = [3400, 3410, 3420, 3430, 3440]

        with tempfile.TemporaryDirectory() as tmpdir:
            json_path = os.path.join(tmpdir, "line_calib.json")
            save_calib_json(min_raw, max_raw, json_path)

            self.assertTrue(os.path.isfile(json_path))
            loaded_min, loaded_max = load_calib_json(json_path)

            self.assertEqual(loaded_min, min_raw)
            self.assertEqual(loaded_max, max_raw)

    def test_write_calib_header(self):
        min_raw = [300, 310, 320, 330, 340]
        max_raw = [3400, 3410, 3420, 3430, 3440]

        with tempfile.TemporaryDirectory() as tmpdir:
            header_path = os.path.join(tmpdir, "app_line_calib_defaults.h")
            write_calib_header(min_raw, max_raw, header_path)

            self.assertTrue(os.path.isfile(header_path))
            with open(header_path, "r", encoding="utf-8") as f:
                content = f.read()
            self.assertIn("{ 300, 310, 320, 330, 340 }", content)


if __name__ == "__main__":
    unittest.main()
