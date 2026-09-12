import unittest
from agv_esp32.tools.agv_ble_gui import TelemetryParser, LineTelemetry, StepPoint

class TestTelemetryParser(unittest.TestCase):
    def setUp(self):
        self.parser = TelemetryParser()

    def test_parse_line_telemetry_valid(self):
        # Format: DATA:LINE,<ms>,<raw0..4>,<norm0..4>,<err>,<line_seen>
        sample = "DATA:LINE,12500,450,480,2850,510,490,50,60,890,70,55,12.5,1"
        result = self.parser.parse_line(sample)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "LINE")
        data: LineTelemetry = result["data"]
        self.assertEqual(data.timestamp_ms, 12500)
        self.assertEqual(len(data.raw_values), 5)
        self.assertEqual(data.raw_values, [450, 480, 2850, 510, 490])
        self.assertEqual(len(data.norm_values), 5)
        self.assertEqual(data.norm_values, [50, 60, 890, 70, 55])
        self.assertAlmostEqual(data.line_error, 12.5, places=2)
        self.assertTrue(data.line_seen)

    def test_parse_line_telemetry_no_line(self):
        sample = "DATA:LINE,12550,200,210,205,202,198,10,12,15,11,9,-99.0,0"
        result = self.parser.parse_line(sample)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "LINE")
        data: LineTelemetry = result["data"]
        self.assertFalse(data.line_seen)
        self.assertAlmostEqual(data.line_error, -99.0, places=2)

    def test_parse_step_data_valid(self):
        # Format: DATA:<ms>,<time_s>,<pwm>,<pct>,<ticks>,<rpm>,<rads>
        sample = "DATA:200,0.200,500,50.0,45,62.5,6.54"
        result = self.parser.parse_line(sample)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "STEP")
        step: StepPoint = result["data"]
        self.assertEqual(step.time_ms, 200)
        self.assertAlmostEqual(step.time_s, 0.2, places=3)
        self.assertEqual(step.pwm, 500)
        self.assertAlmostEqual(step.speed_rpm, 62.5, places=2)

    def test_parse_encoder_response(self):
        sample = "[ENCODER] Left: 1540 | Right: 1542"
        result = self.parser.parse_line(sample)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "ENCODER")
        self.assertEqual(result["left"], 1540)
        self.assertEqual(result["right"], 1542)

    def test_parse_mission_status(self):
        sample = "[MISSION] State: RUNNING, Step: 2"
        result = self.parser.parse_line(sample)
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "MISSION")
        self.assertEqual(result["state"], "RUNNING")
        self.assertEqual(result["step"], 2)

    def test_parse_responses_and_acks(self):
        sample_pong = "[RESP] PONG (STM32 Alive)"
        r_pong = self.parser.parse_line(sample_pong)
        self.assertEqual(r_pong["type"], "RESP")
        self.assertIn("PONG", r_pong["text"])

        sample_ack = "[ACK] MsgID: 0x10, Status: OK (Code: 0)"
        r_ack = self.parser.parse_line(sample_ack)
        self.assertEqual(r_ack["type"], "ACK")

        sample_err = "[ERR] Unknown command: FOO"
        r_err = self.parser.parse_line(sample_err)
        self.assertEqual(r_err["type"], "ERR")

    def test_parse_stm32_log(self):
        sample_log = "[STM32 LOG] Motor L Target: 80.0 RPM"
        r_log = self.parser.parse_line(sample_log)
        self.assertEqual(r_log["type"], "LOG")
        self.assertEqual(r_log["text"], "Motor L Target: 80.0 RPM")

    def test_parse_edge_cases(self):
        self.assertIsNone(self.parser.parse_line(""))
        self.assertIsNone(self.parser.parse_line("   \r\n"))

        # Malformed LINE (not enough tokens)
        res = self.parser.parse_line("DATA:LINE,12500,450,480")
        self.assertEqual(res["type"], "GENERAL")

        # Malformed STEP (invalid numbers)
        res_step = self.parser.parse_line("DATA:abc,def,ghi,jkl,mno,pqr,stu")
        self.assertEqual(res_step["type"], "GENERAL")

        # General unhandled line
        res_gen = self.parser.parse_line("Random boot message from ESP32")
        self.assertEqual(res_gen["type"], "GENERAL")
        self.assertEqual(res_gen["text"], "Random boot message from ESP32")

if __name__ == "__main__":
    unittest.main()
