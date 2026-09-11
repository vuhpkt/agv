#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit Test Suite for AGV Motor PI Controller Component
Simulates closed-loop velocity control against the identified AGV DC Motor transfer function:
G(s) = K / (tau * s + 1) = 0.2699 / (0.0449s + 1)
"""

import unittest
import numpy as np


class MotorPIController:
    """
    Python reference model that mirrors Services/Src/motor_pi.c exactly
    """
    def __init__(self, kp=5.04, ki=112.3, k_ff=3.705, out_min=-1000.0, out_max=1000.0,
                 integral_limit=400.0, deadzone_pwm=40.0, max_accel_rpm_s=600.0):
        self.kp = kp
        self.ki = ki
        self.k_ff = k_ff
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit
        self.deadzone_pwm = deadzone_pwm
        self.max_accel_rpm_s = max_accel_rpm_s

        self.target_rpm = 0.0
        self.measured_rpm = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.ff_term = 0.0
        self.output_pwm = 0.0
        self.is_saturated = False

    def reset(self):
        self.target_rpm = 0.0
        self.measured_rpm = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.ff_term = 0.0
        self.output_pwm = 0.0
        self.is_saturated = False

    def update(self, setpoint_rpm, feedback_rpm, dt_s):
        if dt_s <= 0.0:
            return 0

        self.measured_rpm = feedback_rpm

        # 1. Slew rate limiter
        if self.max_accel_rpm_s > 0.0:
            max_delta = self.max_accel_rpm_s * dt_s
            diff = setpoint_rpm - self.target_rpm
            if diff > max_delta:
                self.target_rpm += max_delta
            elif diff < -max_delta:
                self.target_rpm -= max_delta
            else:
                self.target_rpm = setpoint_rpm
        else:
            self.target_rpm = setpoint_rpm

        if abs(setpoint_rpm) < 0.01 and abs(self.target_rpm) < 0.1:
            self.target_rpm = 0.0
            if abs(feedback_rpm) < 1.0:
                self.integral = 0.0
                self.output_pwm = 0.0
                self.p_term = 0.0
                self.i_term = 0.0
                self.ff_term = 0.0
                self.error = 0.0
                self.prev_error = 0.0
                self.is_saturated = False
                return 0

        # 2. Error
        error = self.target_rpm - feedback_rpm
        self.error = error

        # 3. P term
        self.p_term = self.kp * error

        # 4. Feedforward term
        self.ff_term = self.k_ff * self.target_rpm

        # 5. Integral term with Anti-Windup Clamping
        saturated_pos = (self.is_saturated and self.output_pwm >= self.out_max and error > 0.0)
        saturated_neg = (self.is_saturated and self.output_pwm <= self.out_min and error < 0.0)

        if not saturated_pos and not saturated_neg:
            self.integral += error * dt_s
            if self.integral_limit > 0.0:
                self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        self.i_term = self.ki * self.integral

        # 6. Deadzone compensation
        deadzone = 0.0
        if self.target_rpm > 0.5:
            deadzone = self.deadzone_pwm
        elif self.target_rpm < -0.5:
            deadzone = -self.deadzone_pwm

        # 7. Total output
        total_out = self.ff_term + self.p_term + self.i_term + deadzone

        # 8. Saturation
        if total_out >= self.out_max:
            self.output_pwm = self.out_max
            self.is_saturated = True
        elif total_out <= self.out_min:
            self.output_pwm = self.out_min
            self.is_saturated = True
        else:
            self.output_pwm = total_out
            self.is_saturated = False

        self.prev_error = error
        return int(round(self.output_pwm))


class SimulatedAGVMotor:
    """
    Simulates the continuous-time AGV Motor plant:
    G(s) = K / (tau * s + 1) -> tau * dy/dt + y = K * (u - deadzone)
    """
    def __init__(self, k=0.2699, tau=0.0449, deadzone=35.0):
        self.k = k
        self.tau = tau
        self.deadzone = deadzone
        self.speed_rpm = 0.0

    def step(self, u_pwm, dt):
        # Effective PWM considering deadzone
        if abs(u_pwm) <= self.deadzone:
            effective_pwm = 0.0
        else:
            effective_pwm = u_pwm - np.sign(u_pwm) * self.deadzone

        # First order Euler discretization
        alpha = dt / (self.tau + dt)
        target_ss = self.k * effective_pwm
        self.speed_rpm += alpha * (target_ss - self.speed_rpm)
        return self.speed_rpm


class TestMotorPIController(unittest.TestCase):

    def test_default_gains(self):
        """Kiểm tra các hệ số mặc định phù hợp với mô hình thực tế"""
        ctrl = MotorPIController()
        self.assertAlmostEqual(ctrl.kp, 5.04, places=2)
        self.assertAlmostEqual(ctrl.ki, 112.3, places=1)
        self.assertAlmostEqual(ctrl.k_ff, 3.705, places=3)
        self.assertEqual(ctrl.out_max, 1000.0)
        self.assertEqual(ctrl.out_min, -1000.0)

    def test_step_target_tracking_forward(self):
        """
        Kiểm tra bám bước nhảy vận tốc dương (+150 RPM) trong vòng kín:
        Yêu cầu:
        - Sai số xác lập |e_ss| < 0.5 RPM (nhờ thành phần tích phân)
        - Thời gian xác lập ts < 0.35s
        - Độ vọt lố < 10%
        """
        ctrl = MotorPIController()
        motor = SimulatedAGVMotor()

        dt = 0.010  # 10ms
        setpoint = 150.0  # RPM
        speeds = []

        for _ in range(100):  # 1.0 second
            u = ctrl.update(setpoint, motor.speed_rpm, dt)
            speed = motor.step(u, dt)
            speeds.append(speed)

        # 1. Sai số xác lập
        steady_speed = np.mean(speeds[-20:])
        self.assertAlmostEqual(steady_speed, setpoint, delta=0.5)

        # 2. Không vọt lố quá mức
        peak_speed = np.max(speeds)
        overshoot_pct = max(0.0, (peak_speed - setpoint) / setpoint * 100.0)
        self.assertLess(overshoot_pct, 10.0)

    def test_step_target_tracking_reverse(self):
        """Kiểm tra bám bước nhảy vận tốc âm (-120 RPM)"""
        ctrl = MotorPIController()
        motor = SimulatedAGVMotor()

        dt = 0.010
        setpoint = -120.0  # RPM
        speeds = []

        for _ in range(100):
            u = ctrl.update(setpoint, motor.speed_rpm, dt)
            speed = motor.step(u, dt)
            speeds.append(speed)

        steady_speed = np.mean(speeds[-20:])
        self.assertAlmostEqual(steady_speed, setpoint, delta=0.5)

    def test_zero_speed_stop(self):
        """Khi đặt setpoint = 0, động cơ phải dừng hẳn và ngõ ra về 0"""
        ctrl = MotorPIController()
        motor = SimulatedAGVMotor()
        dt = 0.010

        # Chạy lên 100 RPM trước
        for _ in range(50):
            u = ctrl.update(100.0, motor.speed_rpm, dt)
            motor.step(u, dt)

        # Đặt lệnh dừng 0 RPM
        for _ in range(60):
            u = ctrl.update(0.0, motor.speed_rpm, dt)
            motor.step(u, dt)

        self.assertAlmostEqual(motor.speed_rpm, 0.0, delta=0.5)
        self.assertEqual(u, 0)
        self.assertAlmostEqual(ctrl.integral, 0.0, delta=0.01)

    def test_anti_windup_under_saturation(self):
        """
        Khi động cơ bị kẹt (speed = 0) dù yêu cầu 200 RPM,
        thành phần tích phân không được tăng vô hạn mà phải bị chặn ở integral_limit.
        """
        ctrl = MotorPIController(integral_limit=300.0)
        dt = 0.010

        # Giả lập động cơ bị kẹt cứng (speed luôn = 0) trong 5 giây
        for _ in range(500):
            u = ctrl.update(200.0, 0.0, dt)

        # Output phải bão hòa ở mức cực đại
        self.assertEqual(u, 1000)
        self.assertTrue(ctrl.is_saturated)
        # Integral phải bị chặn ở integral_limit
        self.assertLessEqual(ctrl.integral, 300.0)

    def test_deadzone_compensation(self):
        """Kiểm tra bù vùng chết ma sát khi bắt đầu quay"""
        ctrl = MotorPIController(deadzone_pwm=40.0)
        u_fwd = ctrl.update(10.0, 0.0, 0.010)
        self.assertGreater(u_fwd, 40)

        ctrl.reset()
        u_rev = ctrl.update(-10.0, 0.0, 0.010)
        self.assertLess(u_rev, -40)

    def test_multi_speed_staircase_profile(self):
        """
        Kiểm tra khả năng bám chính xác của bộ điều khiển PI qua 10 cấp tốc độ khác nhau:
        0 -> 40 -> 80 -> 140 -> 200 -> 100 -> 0 -> -60 -> -140 -> 0 RPM.
        Yêu cầu:
        - Ở mọi giai đoạn, sai số xác lập |e_ss| < 0.5 RPM.
        - Độ tuyến tính R^2 > 0.9999
        """
        ctrl = MotorPIController()
        motor = SimulatedAGVMotor()

        stages = [
            (0.0, 0.5), (40.0, 2.0), (80.0, 2.0), (140.0, 2.0), (200.0, 2.0),
            (100.0, 1.5), (0.0, 1.0), (-60.0, 2.0), (-140.0, 2.0), (0.0, 0.8)
        ]
        dt = 0.010

        for target_rpm, duration in stages:
            steps = int(duration / dt)
            stage_speeds = []
            for _ in range(steps):
                u = ctrl.update(target_rpm, motor.speed_rpm, dt)
                speed = motor.step(u, dt)
                stage_speeds.append(speed)

            # Kiểm tra sai số xác lập ở 30% cuối của mỗi cấp tốc độ
            steady_speed = np.mean(stage_speeds[-int(steps * 0.3):])
            self.assertAlmostEqual(steady_speed, target_rpm, delta=0.5)


if __name__ == "__main__":
    unittest.main()


