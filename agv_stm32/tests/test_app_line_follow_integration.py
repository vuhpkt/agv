import os
import re
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_PATH = os.path.join(REPO_ROOT, "Application", "Inc", "app_config.h")
APP_MAIN_PATH = os.path.join(REPO_ROOT, "Application", "Src", "app_main.c")
APP_COMMS_PATH = os.path.join(REPO_ROOT, "Application", "Src", "app_comms.c")
COMMS_PROTO_H = os.path.join(REPO_ROOT, "Services", "Inc", "comms_protocol.h")
COMMS_PROTO_C = os.path.join(REPO_ROOT, "Services", "Src", "comms_protocol.c")


class TestAppLineFollowIntegration(unittest.TestCase):
    def test_app_config_defines_line_follow_parameters(self):
        self.assertTrue(os.path.isfile(CONFIG_PATH), f"Missing {CONFIG_PATH}")
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn("AGV_LINE_FOLLOW_AUTO_START", content,
                      "app_config.h must define AGV_LINE_FOLLOW_AUTO_START")
        self.assertIn("AGV_LINE_FOLLOW_DEFAULT_RPM", content,
                      "app_config.h must define AGV_LINE_FOLLOW_DEFAULT_RPM")
        self.assertIn("AGV_LINE_FOLLOW_ARM_DELAY_MS", content,
                      "app_config.h must define AGV_LINE_FOLLOW_ARM_DELAY_MS")

    def test_app_main_integrates_line_follow(self):
        self.assertTrue(os.path.isfile(APP_MAIN_PATH), f"Missing {APP_MAIN_PATH}")
        with open(APP_MAIN_PATH, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn('#include "app_line_follow.h"', content,
                      "app_main.c must include app_line_follow.h")
        self.assertIn("App_LineFollow_Init", content,
                      "app_main.c must call App_LineFollow_Init()")
        self.assertIn("App_LineFollow_Process", content,
                      "app_main.c must call App_LineFollow_Process()")
        self.assertIn("AGV_LINE_FOLLOW_AUTO_START", content,
                      "app_main.c must handle AGV_LINE_FOLLOW_AUTO_START")

    def test_comms_supports_line_start_and_stop(self):
        with open(COMMS_PROTO_H, "r", encoding="utf-8") as f:
            proto_h = f.read()
        self.assertIn("AGV_MSG_ROUTE_START", proto_h,
                      "comms_protocol.h must define AGV_MSG_ROUTE_START")
        self.assertIn("AGV_MSG_ROUTE_CANCEL", proto_h,
                      "comms_protocol.h must define AGV_MSG_ROUTE_CANCEL")

        with open(COMMS_PROTO_C, "r", encoding="utf-8") as f:
            proto_c = f.read()
        self.assertIn("agv_pack_route_start", proto_c,
                      "comms_protocol.c must implement agv_pack_route_start")
        self.assertIn("agv_pack_route_cancel", proto_c,
                      "comms_protocol.c must implement agv_pack_route_cancel")

        with open(APP_COMMS_PATH, "r", encoding="utf-8") as f:
            comms_c = f.read()
        self.assertIn("AGV_MSG_ROUTE_START", comms_c,
                      "app_comms.c must handle AGV_MSG_ROUTE_START")
        self.assertIn("AGV_MSG_ROUTE_CANCEL", comms_c,
                      "app_comms.c must handle AGV_MSG_ROUTE_CANCEL")


if __name__ == "__main__":
    unittest.main()
