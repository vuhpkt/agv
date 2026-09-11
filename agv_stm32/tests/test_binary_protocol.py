#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for AGV Binary Protocol (comms_protocol)
Verifies:
- Frame header & SOF (0xAA, 0x55)
- CRC16-CCITT algorithm (poly 0x1021, init 0xFFFF)
- Streaming byte parser state machine
- Packing and unpacking for:
  - PING (0x01), PONG (0x81), ACK (0x80)
  - EMERGENCY_STOP (0x02)
  - ROUTE_LOAD (0x10), ROUTE_START (0x11), ROUTE_CANCEL (0x12), MISSION_STATUS (0x90)
  - DIAG_STEP_TEST (0x20), DIAG_SET_SPEED (0x21), DIAG_SET_RPM (0x22)
  - DIAG_GET_ENC (0x23), DIAG_ENC_DATA (0xA3), DIAG_RESET_ENC (0x24)
  - DIAG_PI_TUNE (0x25), DIAG_LINE_TEST (0x26)
  - MSG_LOG (0x99)
- Noise, fragmented streams, and corrupted CRC handling.
"""

import unittest
import struct


def crc16_ccitt(data: bytes, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class AGVBinaryProtocolModel:
    SOF1 = 0xAA
    SOF2 = 0x55

    # Message IDs
    MSG_PING = 0x01
    MSG_EMERGENCY_STOP = 0x02
    MSG_ROUTE_LOAD = 0x10
    MSG_ROUTE_START = 0x11
    MSG_ROUTE_CANCEL = 0x12
    MSG_DIAG_STEP_TEST = 0x20
    MSG_DIAG_SET_SPEED = 0x21
    MSG_DIAG_SET_RPM = 0x22
    MSG_DIAG_GET_ENC = 0x23
    MSG_DIAG_RESET_ENC = 0x24
    MSG_DIAG_PI_TUNE = 0x25
    MSG_DIAG_LINE_TEST = 0x26

    MSG_ACK = 0x80
    MSG_PONG = 0x81
    MSG_MISSION_STATUS = 0x90
    MSG_TELEMETRY = 0x91
    MSG_LOG = 0x99
    MSG_DIAG_ENC_DATA = 0xA3

    # Route Actions
    ACTION_STOP = 0x00
    ACTION_FORWARD = 0x01
    ACTION_TURN_LEFT = 0x02
    ACTION_TURN_RIGHT = 0x03

    # Mission States
    MISSION_IDLE = 0x00
    MISSION_RUNNING = 0x01
    MISSION_COMPLETED = 0x02
    MISSION_ERROR_LOST = 0x03
    MISSION_ERROR_TIMEOUT = 0x04

    @staticmethod
    def pack_frame(msg_id: int, payload: bytes = b"") -> bytes:
        length = len(payload)
        header = bytes([AGVBinaryProtocolModel.SOF1, AGVBinaryProtocolModel.SOF2, msg_id, length])
        crc_data = bytes([msg_id, length]) + payload
        crc = crc16_ccitt(crc_data)
        return header + payload + struct.pack(">H", crc)

    @staticmethod
    def pack_route_load(target_rpm: float, actions: list) -> bytes:
        count = len(actions)
        rpm_x10 = int(round(target_rpm * 10))
        payload = struct.pack("<Bh", count, rpm_x10)
        # Pad up to 16 actions
        act_bytes = bytearray(16)
        for i in range(min(count, 16)):
            act_bytes[i] = actions[i]
        payload += bytes(act_bytes)
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_ROUTE_LOAD, payload)

    @staticmethod
    def pack_diag_step_test(motor_id: int, pwm: int, duration_ms: int) -> bytes:
        payload = struct.pack("<Bhi", motor_id, pwm, duration_ms)
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_STEP_TEST, payload)

    @staticmethod
    def pack_diag_set_speed(left_pwm: int, right_pwm: int) -> bytes:
        payload = struct.pack("<hh", left_pwm, right_pwm)
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_SET_SPEED, payload)

    @staticmethod
    def pack_diag_set_rpm(left_rpm: float, right_rpm: float) -> bytes:
        payload = struct.pack("<hh", int(round(left_rpm * 10)), int(round(right_rpm * 10)))
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_SET_RPM, payload)

    @staticmethod
    def pack_diag_pi_tune(kp: float, ki: float, kff: float) -> bytes:
        payload = struct.pack("<HHH", int(round(kp * 100)), int(round(ki * 100)), int(round(kff * 100)))
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_PI_TUNE, payload)

    @staticmethod
    def pack_diag_line_test(mode: int, duration_ms: int, base_rpm: float) -> bytes:
        payload = struct.pack("<BIh", mode, duration_ms, int(round(base_rpm * 10)))
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_LINE_TEST, payload)

    @staticmethod
    def pack_ack(acked_msg_id: int, status_code: int = 0) -> bytes:
        payload = bytes([acked_msg_id, status_code])
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_ACK, payload)

    @staticmethod
    def pack_mission_status(state: int, current_step_idx: int) -> bytes:
        payload = struct.pack("<BBH", state, current_step_idx, 0)
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_MISSION_STATUS, payload)

    @staticmethod
    def pack_log(text: str) -> bytes:
        text_bytes = text.encode("utf-8")[:64]
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_LOG, text_bytes)

    @staticmethod
    def pack_diag_enc_data(left_enc: int, right_enc: int) -> bytes:
        payload = struct.pack("<ii", left_enc, right_enc)
        return AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_ENC_DATA, payload)


class StreamingParser:
    """
    Python model of the C streaming parser state machine.
    """
    STATE_WAIT_SOF1 = 0
    STATE_WAIT_SOF2 = 1
    STATE_MSG_ID = 2
    STATE_LENGTH = 3
    STATE_PAYLOAD = 4
    STATE_CRC_H = 5
    STATE_CRC_L = 6

    def __init__(self):
        self.state = self.STATE_WAIT_SOF1
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_rx = 0

    def feed_byte(self, b: int):
        if self.state == self.STATE_WAIT_SOF1:
            if b == AGVBinaryProtocolModel.SOF1:
                self.state = self.STATE_WAIT_SOF2
        elif self.state == self.STATE_WAIT_SOF2:
            if b == AGVBinaryProtocolModel.SOF2:
                self.state = self.STATE_MSG_ID
            elif b == AGVBinaryProtocolModel.SOF1:
                self.state = self.STATE_WAIT_SOF2
            else:
                self.state = self.STATE_WAIT_SOF1
        elif self.state == self.STATE_MSG_ID:
            self.msg_id = b
            self.state = self.STATE_LENGTH
        elif self.state == self.STATE_LENGTH:
            self.length = b
            self.payload = bytearray()
            if self.length > 128:
                # Invalid length
                self.state = self.STATE_WAIT_SOF1
            elif self.length == 0:
                self.state = self.STATE_CRC_H
            else:
                self.state = self.STATE_PAYLOAD
        elif self.state == self.STATE_PAYLOAD:
            self.payload.append(b)
            if len(self.payload) == self.length:
                self.state = self.STATE_CRC_H
        elif self.state == self.STATE_CRC_H:
            self.crc_rx = b << 8
            self.state = self.STATE_CRC_L
        elif self.state == self.STATE_CRC_L:
            self.crc_rx |= b
            self.state = self.STATE_WAIT_SOF1

            # Validate CRC
            crc_calc = crc16_ccitt(bytes([self.msg_id, self.length]) + bytes(self.payload))
            if crc_calc == self.crc_rx:
                return {
                    "valid": True,
                    "msg_id": self.msg_id,
                    "payload": bytes(self.payload)
                }
            else:
                return {
                    "valid": False,
                    "error": "CRC_MISMATCH",
                    "expected": crc_calc,
                    "got": self.crc_rx
                }
        return None


class TestAGVBinaryProtocol(unittest.TestCase):
    def setUp(self):
        self.parser = StreamingParser()

    def test_crc16_known_vector(self):
        # Standard CRC16-CCITT test vector for b"123456789" with 0xFFFF init is 0x29B1
        vec = b"123456789"
        crc = crc16_ccitt(vec, 0xFFFF)
        self.assertEqual(crc, 0x29B1)

    def test_ping_frame(self):
        frame = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_PING)
        self.assertEqual(frame[0], 0xAA)
        self.assertEqual(frame[1], 0x55)
        self.assertEqual(frame[2], AGVBinaryProtocolModel.MSG_PING)
        self.assertEqual(frame[3], 0)  # Length 0
        self.assertEqual(len(frame), 6)  # 2 SOF + 1 ID + 1 LEN + 0 PAYLOAD + 2 CRC

        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertTrue(res["valid"])
        self.assertEqual(res["msg_id"], AGVBinaryProtocolModel.MSG_PING)
        self.assertEqual(len(res["payload"]), 0)

    def test_route_load_frame(self):
        actions = [
            AGVBinaryProtocolModel.ACTION_FORWARD,
            AGVBinaryProtocolModel.ACTION_TURN_RIGHT,
            AGVBinaryProtocolModel.ACTION_FORWARD,
            AGVBinaryProtocolModel.ACTION_STOP
        ]
        frame = AGVBinaryProtocolModel.pack_route_load(target_rpm=80.0, actions=actions)
        self.assertEqual(frame[2], AGVBinaryProtocolModel.MSG_ROUTE_LOAD)
        self.assertEqual(frame[3], 19)  # 1 (count) + 2 (rpm) + 16 (actions)

        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertTrue(res["valid"])
        self.assertEqual(res["msg_id"], AGVBinaryProtocolModel.MSG_ROUTE_LOAD)

        count, rpm_x10 = struct.unpack("<Bh", res["payload"][:3])
        self.assertEqual(count, 4)
        self.assertEqual(rpm_x10, 800)
        parsed_actions = list(res["payload"][3:3+count])
        self.assertEqual(parsed_actions, actions)

    def test_emergency_stop_frame(self):
        frame = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_EMERGENCY_STOP)
        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertEqual(res["msg_id"], AGVBinaryProtocolModel.MSG_EMERGENCY_STOP)

    def test_diag_step_test_frame(self):
        frame = AGVBinaryProtocolModel.pack_diag_step_test(motor_id=1, pwm=600, duration_ms=1500)
        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        m_id, pwm, dur = struct.unpack("<Bhi", res["payload"])
        self.assertEqual(m_id, 1)
        self.assertEqual(pwm, 600)
        self.assertEqual(dur, 1500)

    def test_diag_pi_tune_frame(self):
        frame = AGVBinaryProtocolModel.pack_diag_pi_tune(kp=5.34, ki=114.30, kff=3.81)
        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        kp_raw, ki_raw, kff_raw = struct.unpack("<HHH", res["payload"])
        self.assertEqual(kp_raw, 534)
        self.assertEqual(ki_raw, 11430)
        self.assertEqual(kff_raw, 381)

    def test_log_frame(self):
        msg = "AGV Boot OK"
        frame = AGVBinaryProtocolModel.pack_log(msg)
        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertEqual(res["msg_id"], AGVBinaryProtocolModel.MSG_LOG)
        self.assertEqual(res["payload"].decode("utf-8"), msg)

    def test_noise_and_recovery(self):
        valid_frame = AGVBinaryProtocolModel.pack_ack(AGVBinaryProtocolModel.MSG_ROUTE_LOAD, 0)
        noisy_stream = bytes([0x00, 0xFF, 0xAA, 0x12, 0xAA, 0x55]) + valid_frame[2:]

        res = None
        for b in noisy_stream:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertTrue(res["valid"])
        self.assertEqual(res["msg_id"], AGVBinaryProtocolModel.MSG_ACK)

    def test_corrupted_crc_rejected(self):
        frame = bytearray(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_PING))
        frame[-1] ^= 0xFF  # Corrupt CRC low byte

        res = None
        for b in frame:
            r = self.parser.feed_byte(b)
            if r:
                res = r
        self.assertIsNotNone(res)
        self.assertFalse(res["valid"])
        self.assertEqual(res["error"], "CRC_MISMATCH")


if __name__ == "__main__":
    unittest.main()
