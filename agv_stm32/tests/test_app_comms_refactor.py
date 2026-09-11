#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for App Comms Dispatcher Refactoring
Verifies that app_comms acts as a clean Dispatcher, delegating parsing
to comms_protocol and routing via structured switch dispatch on binary frame IDs.
"""

import unittest
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestAppCommsRefactor(unittest.TestCase):

    def test_app_comms_uses_comms_protocol(self):
        """app_comms.c must include comms_protocol.h and call agv_parser_feed_byte"""
        file_path = os.path.join(REPO_ROOT, "Application", "Src", "app_comms.c")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            '#include "comms_protocol.h"',
            content,
            "app_comms.c must include comms_protocol.h"
        )
        self.assertIn(
            "agv_parser_feed_byte",
            content,
            "app_comms.c must call agv_parser_feed_byte"
        )

    def test_app_comms_uses_switch_dispatch(self):
        """app_comms.c must dispatch commands via switch statement on binary frame msg_id"""
        file_path = os.path.join(REPO_ROOT, "Application", "Src", "app_comms.c")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        self.assertIn(
            "switch (frame->msg_id)",
            content,
            "app_comms.c should use switch dispatch on frame->msg_id"
        )
        self.assertIn(
            "AGV_MSG_PING",
            content,
            "app_comms.c should handle AGV_MSG_PING"
        )
        self.assertIn(
            "AGV_MSG_ROUTE_LOAD",
            content,
            "app_comms.c should handle AGV_MSG_ROUTE_LOAD"
        )
        self.assertIn(
            "AGV_MSG_EMERGENCY_STOP",
            content,
            "app_comms.c should handle AGV_MSG_EMERGENCY_STOP"
        )
        self.assertIn(
            "AGV_MSG_DIAG_SET_SPEED",
            content,
            "app_comms.c should handle AGV_MSG_DIAG_SET_SPEED"
        )


if __name__ == "__main__":
    unittest.main()
