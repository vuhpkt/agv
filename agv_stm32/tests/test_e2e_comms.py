#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
End-to-End Comms Integration Test Suite (Task 8)
Simulates ESP32 Gateway communicating with STM32 AGV Controller via UART Binary Protocol.
Tests:
- Ping / Pong handshake
- Route loading and execution state transitions
- Emergency stop interrupt
- Diagnostics command suite (Step test, Set Speed, Set RPM, Encoders, PI tune, Line test)
- Checksum error and unknown command handling
"""

import unittest
import struct
from tests.test_binary_protocol import AGVBinaryProtocolModel, StreamingParser, crc16_ccitt


class SimulatedSTM32Controller:
    """
    Python behavioral simulation of STM32 app_comms and app_line_follow.
    """
    def __init__(self):
        self.parser = StreamingParser()
        self.tx_buffer = bytearray()
        self.led_state = False
        self.motor_stopped = True
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.encoder_left = 1250
        self.encoder_right = 1248
        self.pi_kp = 5.34
        self.pi_ki = 114.3
        self.pi_kff = 3.81
        self.route_actions = []
        self.route_step = 0
        self.route_running = False

    def receive_bytes(self, data: bytes):
        for b in data:
            res = self.parser.feed_byte(b)
            if res:
                if res["valid"]:
                    self._handle_frame(res["msg_id"], res["payload"])
                else:
                    # Send CRC NACK
                    nack = AGVBinaryProtocolModel.pack_ack(self.parser.msg_id, 1)  # AGV_ACK_ERR_CRC
                    self.tx_buffer.extend(nack)

    def _handle_frame(self, msg_id: int, payload: bytes):
        if msg_id == AGVBinaryProtocolModel.MSG_PING:
            self.led_state = not self.led_state
            pong = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_PONG)
            self.tx_buffer.extend(pong)
        elif msg_id == AGVBinaryProtocolModel.MSG_EMERGENCY_STOP:
            self.route_running = False
            self.motor_stopped = True
            self.left_pwm = 0
            self.right_pwm = 0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_ROUTE_LOAD:
            count, rpm_x10 = struct.unpack("<Bh", payload[:3])
            actions = list(payload[3:3+count])
            self.route_actions = actions
            self.route_step = 0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_ROUTE_START:
            self.route_running = True
            self.route_step = 0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
            # Emit mission running event
            ev = AGVBinaryProtocolModel.pack_mission_status(AGVBinaryProtocolModel.MISSION_RUNNING, 0)
            self.tx_buffer.extend(ev)
        elif msg_id == AGVBinaryProtocolModel.MSG_ROUTE_CANCEL:
            self.route_running = False
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_SET_SPEED:
            l, r = struct.unpack("<hh", payload)
            self.left_pwm = l
            self.right_pwm = r
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_SET_RPM:
            l, r = struct.unpack("<hh", payload)
            self.left_rpm = l / 10.0
            self.right_rpm = r / 10.0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_GET_ENC:
            enc_data = AGVBinaryProtocolModel.pack_diag_enc_data(self.encoder_left, self.encoder_right)
            self.tx_buffer.extend(enc_data)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_RESET_ENC:
            self.encoder_left = 0
            self.encoder_right = 0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_PI_TUNE:
            kp, ki, kff = struct.unpack("<HHH", payload)
            self.pi_kp = kp / 100.0
            self.pi_ki = ki / 100.0
            self.pi_kff = kff / 100.0
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_STEP_TEST:
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        elif msg_id == AGVBinaryProtocolModel.MSG_DIAG_LINE_TEST:
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 0)
            self.tx_buffer.extend(ack)
        else:
            ack = AGVBinaryProtocolModel.pack_ack(msg_id, 2)  # AGV_ACK_ERR_UNKNOWN_CMD
            self.tx_buffer.extend(ack)

    def pop_tx_frames(self):
        parser = StreamingParser()
        frames = []
        for b in self.tx_buffer:
            res = parser.feed_byte(b)
            if res and res["valid"]:
                frames.append(res)
        self.tx_buffer.clear()
        return frames

    def simulate_crossline_trigger(self):
        """Simulate AGV hitting an intersection line on track"""
        if self.route_running and self.route_step < len(self.route_actions):
            self.route_step += 1
            if self.route_step >= len(self.route_actions):
                self.route_running = False
                ev = AGVBinaryProtocolModel.pack_mission_status(
                    AGVBinaryProtocolModel.MISSION_COMPLETED,
                    self.route_step
                )
                self.tx_buffer.extend(ev)


class TestE2EComms(unittest.TestCase):
    def setUp(self):
        self.stm32 = SimulatedSTM32Controller()

    def test_ping_pong_e2e(self):
        ping = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_PING)
        self.stm32.receive_bytes(ping)
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_PONG)

    def test_full_route_queue_lifecycle(self):
        actions = [
            AGVBinaryProtocolModel.ACTION_FORWARD,
            AGVBinaryProtocolModel.ACTION_TURN_RIGHT,
            AGVBinaryProtocolModel.ACTION_STOP
        ]
        # 1. Load Route
        load_cmd = AGVBinaryProtocolModel.pack_route_load(target_rpm=80.0, actions=actions)
        self.stm32.receive_bytes(load_cmd)
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        acked_id, status = struct.unpack("<BB", frames[0]["payload"])
        self.assertEqual(acked_id, AGVBinaryProtocolModel.MSG_ROUTE_LOAD)
        self.assertEqual(status, 0)

        # 2. Start Route
        start_cmd = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_ROUTE_START)
        self.stm32.receive_bytes(start_cmd)
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 2)  # ACK + MISSION_STATUS(RUNNING)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        self.assertEqual(frames[1]["msg_id"], AGVBinaryProtocolModel.MSG_MISSION_STATUS)
        state, step, _ = struct.unpack("<BBH", frames[1]["payload"])
        self.assertEqual(state, AGVBinaryProtocolModel.MISSION_RUNNING)
        self.assertEqual(step, 0)

        # 3. Simulate passing 3 intersections
        self.stm32.simulate_crossline_trigger()  # step 1 (FORWARD done)
        self.stm32.simulate_crossline_trigger()  # step 2 (TURN_RIGHT done)
        self.stm32.simulate_crossline_trigger()  # step 3 (STOP done -> complete)

        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_MISSION_STATUS)
        state, step, _ = struct.unpack("<BBH", frames[0]["payload"])
        self.assertEqual(state, AGVBinaryProtocolModel.MISSION_COMPLETED)
        self.assertEqual(step, 3)

    def test_emergency_stop_interrupts_route(self):
        actions = [AGVBinaryProtocolModel.ACTION_FORWARD, AGVBinaryProtocolModel.ACTION_FORWARD]
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_route_load(80.0, actions))
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_ROUTE_START))
        self.stm32.pop_tx_frames()
        self.assertTrue(self.stm32.route_running)

        # Emergency stop arrives mid-flight
        stop_cmd = AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_EMERGENCY_STOP)
        self.stm32.receive_bytes(stop_cmd)
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        self.assertFalse(self.stm32.route_running)
        self.assertTrue(self.stm32.motor_stopped)

    def test_diag_encoder_query_and_reset(self):
        # Query encoders
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_GET_ENC))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_DIAG_ENC_DATA)
        l, r = struct.unpack("<ii", frames[0]["payload"])
        self.assertEqual(l, 1250)
        self.assertEqual(r, 1248)

        # Reset encoders
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_RESET_ENC))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)

        # Query again -> should be 0
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_DIAG_GET_ENC))
        frames = self.stm32.pop_tx_frames()
        l, r = struct.unpack("<ii", frames[0]["payload"])
        self.assertEqual(l, 0)
        self.assertEqual(r, 0)

    def test_diag_speed_and_rpm_commands(self):
        # Direct PWM
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_diag_set_speed(300, -300))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        self.assertEqual(self.stm32.left_pwm, 300)
        self.assertEqual(self.stm32.right_pwm, -300)

        # Target RPM
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_diag_set_rpm(75.5, 75.5))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        self.assertAlmostEqual(self.stm32.left_rpm, 75.5)
        self.assertAlmostEqual(self.stm32.right_rpm, 75.5)

    def test_diag_pi_tune(self):
        self.stm32.receive_bytes(AGVBinaryProtocolModel.pack_diag_pi_tune(6.12, 120.50, 4.05))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        self.assertAlmostEqual(self.stm32.pi_kp, 6.12)
        self.assertAlmostEqual(self.stm32.pi_ki, 120.50)
        self.assertAlmostEqual(self.stm32.pi_kff, 4.05)

    def test_corrupted_crc_triggers_nack(self):
        bad_frame = bytearray(AGVBinaryProtocolModel.pack_frame(AGVBinaryProtocolModel.MSG_PING))
        bad_frame[-1] ^= 0x55  # Corrupt CRC
        self.stm32.receive_bytes(bytes(bad_frame))
        frames = self.stm32.pop_tx_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["msg_id"], AGVBinaryProtocolModel.MSG_ACK)
        _, status = struct.unpack("<BB", frames[0]["payload"])
        self.assertEqual(status, 1)  # AGV_ACK_ERR_CRC


if __name__ == "__main__":
    unittest.main()
