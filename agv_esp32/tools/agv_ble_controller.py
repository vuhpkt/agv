#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGV ESP32 - Wireless BLE Diagnostics Workbench & Interactive Controller
========================================================================
Connects to AGV ESP32 gateway over Bluetooth Low Energy (NimBLE NUS),
providing an interactive command REPL, real-time IR sensor array monitor,
optical desk calibration assistant, and motor step response data acquisition.

Usage:
  # 1. Interactive REPL prompt:
  python agv_ble_controller.py

  # 2. Standalone IR sensor array monitor (default 15000 ms):
  python agv_ble_controller.py --monitor-ir 15000

  # 3. Standalone IR desk calibration & header generator (default 10000 ms):
  python agv_ble_controller.py --calibrate 10000

  # 4. Motor step response test & plot:
  python agv_ble_controller.py --step L 600 1500 --plot

  # 5. Scan available BLE devices:
  python agv_ble_controller.py --scan
"""

import argparse
import asyncio
import csv
import datetime
import json
import os
import re
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

# Ensure stdout handles UTF-8 on Windows
if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# Nordic UART Service (NUS) UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Central writes to Peripheral
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Peripheral notifies Central
DEFAULT_DEVICE_NAME = "AGV_ESP32"

# Telemetry regex patterns
LINE_DATA_PATTERN = re.compile(
    r"^DATA:LINE,(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(-?\d+),([01])"
)
STEP_DATA_PATTERN = re.compile(
    r"^DATA:(\d+),([0-9.]+),(-?\d+),(-?[0-9.]+),(-?\d+),(-?[0-9.]+)(?:,(-?[0-9.]+))?"
)

SENSOR_NAMES = ["IR_R2 (Ch0)", "IR_R1 (Ch1)", "IR_MID(Ch2)", "IR_L1 (Ch3)", "IR_L2 (Ch4)"]

# Terminal ANSI Color codes
COLOR_RESET = "\033[0m"
COLOR_BOLD = "\033[1m"
COLOR_GREEN = "\033[32m"
COLOR_RED = "\033[31m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_MAGENTA = "\033[35m"
COLOR_CYAN = "\033[36m"


def colorize(text, color_code):
    """Wraps text in ANSI color codes if supported."""
    if sys.stdout.isatty():
        return f"{color_code}{text}{COLOR_RESET}"
    return text


def render_bar(val, max_val=1000, width=20):
    """
    Renders a Unicode visual progress bar |██████░░░░|.
    """
    clamped = max(0, min(val, max_val))
    fill = int((clamped / max_val) * width)
    return "█" * fill + "░" * (width - fill)


def parse_line_telemetry(line):
    """
    Extracts telemetry fields from DATA:LINE string.
    Returns dictionary with parsed fields or None if match fails.
    """
    m = LINE_DATA_PATTERN.match(line.strip())
    if not m:
        return None
    return {
        "elapsed_ms": int(m.group(1)),
        "raw": [int(m.group(i)) for i in range(2, 7)],
        "norm": [int(m.group(i)) for i in range(7, 12)],
        "err": int(m.group(12)),
        "line_seen": int(m.group(13)) == 1,
    }


def parse_step_telemetry(line):
    """
    Extracts step response fields from DATA:<ms>,<time_s>,<pwm>,<pct>,<ticks>,<rpm>,<rads>.
    Returns dictionary with parsed fields or None if match fails.
    """
    m = STEP_DATA_PATTERN.match(line.strip())
    if not m:
        return None
    rpm = float(m.group(6))
    rads = float(m.group(7)) if m.group(7) else rpm * (2.0 * 3.1415926535 / 60.0)
    return {
        "elapsed_ms": int(m.group(1)),
        "time_s": float(m.group(2)),
        "pwm": int(m.group(3)),
        "pwm_pct": float(m.group(4)),
        "delta_ticks": int(m.group(5)),
        "rpm": rpm,
        "rads": rads,
    }


class CalibrationStats:
    """Accumulates running min and max raw ADC values across 5 sensors."""

    def __init__(self, num_sensors=5):
        self.num_sensors = num_sensors
        self.min_raw = [4095] * num_sensors
        self.max_raw = [0] * num_sensors
        self.sample_count = 0

    def update(self, raw_values):
        if len(raw_values) != self.num_sensors:
            return
        for i in range(self.num_sensors):
            val = raw_values[i]
            if val < self.min_raw[i]:
                self.min_raw[i] = val
            if val > self.max_raw[i]:
                self.max_raw[i] = val
        self.sample_count += 1


def validate_calibration(min_raw, max_raw, min_dynamic_range=100):
    """
    Checks that min < max and max - min >= min_dynamic_range for all sensors.
    """
    errors = []
    num_sensors = min(len(min_raw), len(max_raw))

    for i in range(num_sensors):
        name = SENSOR_NAMES[i] if i < len(SENSOR_NAMES) else f"Sensor {i}"
        mn = min_raw[i]
        mx = max_raw[i]
        delta = mx - mn

        if mn < 0 or mx > 4095:
            errors.append(f"{name}: ADC reading out of bounds [0..4095] (min={mn}, max={mx})")
        elif mn >= mx:
            errors.append(f"{name}: Min ADC ({mn}) >= Max ADC ({mx})")
        elif delta < min_dynamic_range:
            errors.append(f"{name}: Dynamic range too low ({delta} < {min_dynamic_range} ADC units)")

    return len(errors) == 0, errors


def generate_header_content(min_raw, max_raw):
    """Generates C header file app_line_calib_defaults.h content."""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    min_str = ", ".join(str(v) for v in min_raw)
    max_str = ", ".join(str(v) for v in max_raw)

    return f"""/**
 * @file app_line_calib_defaults.h
 * @brief Default optical calibration parameters for 5-sensor IR array
 *
 * Auto-generated by tools/agv_ble_controller.py on {timestamp}
 * Values represent ADC readings for white background (min) and black line (max).
 */

#ifndef APP_LINE_CALIB_DEFAULTS_H
#define APP_LINE_CALIB_DEFAULTS_H

#include <stdint.h>

/**
 * @brief Calibrated raw ADC thresholds for 5 IR channels [R2, R1, MID, L1, L2]
 */
#define APP_LINE_CALIB_MIN_RAW   {{ {min_str} }}
#define APP_LINE_CALIB_MAX_RAW   {{ {max_str} }}

#endif /* APP_LINE_CALIB_DEFAULTS_H */
"""


def translate_cli_command(user_input):
    """
    Normalizes interactive user commands to uppercase protocol commands.
    """
    line = user_input.strip()
    if not line:
        return ""

    parts = line.split(None, 1)
    cmd = parts[0].upper()
    args = parts[1] if len(parts) > 1 else ""

    # Aliases
    if cmd in ("STOP", "EMG"):
        return "STOP"
    elif cmd in ("STEP", "STEP_TEST"):
        return f"STEP_TEST {args}".strip()
    elif cmd in ("CALIB", "CALIBRATE", "LINE_CALIB"):
        return f"LINE_CALIB {args}".strip()
    elif cmd in ("MONITOR", "MONITOR_IR"):
        return f"MONITOR_IR {args}".strip()
    elif cmd in ("SET_SPEED", "SPEED"):
        return f"SET_SPEED {args}".strip()
    elif cmd in ("SET_RPM", "RPM"):
        return f"SET_RPM {args}".strip()
    elif cmd in ("ENC", "GET_ENC"):
        return "GET_ENC"
    elif cmd in ("RESET_ENC", "RESET"):
        return "RESET_ENC"
    elif cmd in ("PI_TUNE", "PI"):
        return f"PI_TUNE {args}".strip()
    elif cmd in ("LINE_TEST",):
        return f"LINE_TEST {args}".strip()
    elif cmd in ("ROUTE_LOAD",):
        return f"ROUTE_LOAD {args}".strip()
    elif cmd in ("ROUTE_START",):
        return "ROUTE_START"
    elif cmd in ("ROUTE_CANCEL",):
        return "ROUTE_CANCEL"
    elif cmd in ("ROUTE_TEST",):
        return "ROUTE_TEST"
    elif cmd in ("PING",):
        return "PING"
    elif cmd in ("HELP", "?"):
        return "HELP"

    return f"{cmd} {args}".strip()


class AGVBleClient:
    """Async BLE Client for AGV ESP32 Nordic UART Service."""

    def __init__(self, device_name=DEFAULT_DEVICE_NAME, address=None):
        self.device_name = device_name
        self.address = address
        self.client = None
        self._rx_buffer = ""
        self._line_listeners = []
        self._raw_listeners = []

    def add_line_listener(self, callback):
        self._line_listeners.append(callback)

    def remove_line_listener(self, callback):
        if callback in self._line_listeners:
            self._line_listeners.remove(callback)

    async def connect(self, timeout=10.0):
        if BleakClient is None:
            raise RuntimeError("bleak library is not installed. Run 'pip install bleak'")

        target_device = None
        if self.address:
            print(f"[+] Connecting directly to BLE address: {self.address}...")
            target_device = self.address
        else:
            print(f"[+] Scanning for BLE device '{self.device_name}' (timeout: {timeout}s)...")
            devices = await BleakScanner.discover(timeout=timeout)
            for d in devices:
                if d.name and self.device_name.lower() in d.name.lower():
                    target_device = d
                    break

        if not target_device:
            raise RuntimeError(f"Could not find device '{self.device_name}'. Ensure AGV ESP32 is powered on.")

        self.client = BleakClient(target_device)
        await self.client.connect()
        print(f"[+] Connected to AGV ESP32 ({self.client.address})!")

        # Subscribe to TX Characteristic notifications
        await self.client.start_notify(NUS_TX_CHAR_UUID, self._on_notification)
        print("[+] Subscribed to BLE NUS notifications.")

    async def disconnect(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(NUS_TX_CHAR_UUID)
            except Exception:
                pass
            await self.client.disconnect()
            print("[+] Disconnected from AGV ESP32.")

    async def send_command(self, cmd_text):
        """Sends an ASCII text command to ESP32 RX characteristic."""
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE client is not connected")

        if not cmd_text.endswith("\n"):
            cmd_text += "\n"

        await self.client.write_gatt_char(NUS_RX_CHAR_UUID, cmd_text.encode("utf-8"), response=False)

    def _on_notification(self, sender, data: bytearray):
        """Handles incoming BLE chunk stream and splits complete lines."""
        text = data.decode("utf-8", errors="replace")
        self._rx_buffer += text

        lines = self._rx_buffer.split("\n")
        self._rx_buffer = lines.pop()

        for line in lines:
            line = line.strip("\r")
            if not line:
                continue
            for listener in list(self._line_listeners):
                try:
                    listener(line)
                except Exception as e:
                    print(f"[-] Error in line listener: {e}", file=sys.stderr)


# =============================================================================
# High-Level Diagnostic Workflows
# =============================================================================

async def run_ir_monitor_workflow(client: AGVBleClient, duration_ms: int = 15000):
    """
    Renders a live 20Hz Unicode graphical view of the 5 IR sensors.
    """
    print(f"\n[+] Starting IR Array Monitor for {duration_ms} ms (Ctrl+C to stop)...")
    queue = asyncio.Queue()

    def on_line(line):
        parsed = parse_line_telemetry(line)
        if parsed:
            queue.put_nowait(parsed)
        elif line.startswith("[STM32 LOG]") or line.startswith("[RESP]"):
            print(f"  {line}")

    client.add_line_listener(on_line)
    await client.send_command(f"MONITOR_IR {duration_ms}")

    start_time = time.time()
    try:
        while (time.time() - start_time) * 1000 < duration_ms + 1000:
            try:
                item = await asyncio.wait_for(queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue

            elapsed_ms = item["elapsed_ms"]
            raw = item["raw"]
            norm = item["norm"]
            err = item["err"]
            line_seen = item["line_seen"]

            # Clear screen and redraw
            sys.stdout.write("\033[H\033[J")
            seen_str = colorize("YES [ON TRACK]", COLOR_GREEN) if line_seen else colorize("NO [LOST]", COLOR_RED)
            err_str = f"{err:>4d}"
            print("==================================================")
            print("  AGV STM32 IR ARRAY MONITOR (BLE WIRELESS)")
            print(f"  Elapsed: {elapsed_ms} ms | Line Detected: {seen_str}")
            print(f"  Centroid Error (e_line): {err_str} (Target: 0)")
            print("==================================================")
            for idx in range(5):
                bar = render_bar(norm[idx], max_val=1000, width=22)
                color = COLOR_GREEN if norm[idx] > 500 else COLOR_CYAN
                bar_colored = colorize(bar, color)
                print(f"  {SENSOR_NAMES[idx]} |{bar_colored}| Norm: {norm[idx]:>4d} | Raw ADC: {raw[idx]:>4d}")
            print("==================================================")
            print("  Press Ctrl+C or send 'STOP' to exit.")
            sys.stdout.flush()

    except asyncio.CancelledError:
        pass
    finally:
        client.remove_line_listener(on_line)
        print("\n[+] IR Monitor finished.")


async def run_calibration_workflow(
    client: AGVBleClient,
    duration_ms: int = 10000,
    save: bool = True,
    header_path: str = None,
    json_path: str = None,
):
    """
    Tracks min/max raw ADC while swiping paper/tape, checks dynamic range,
    and updates C header.
    """
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if header_path is None:
        header_path = os.path.join(repo_root, "agv_stm32", "Application", "Inc", "app_line_calib_defaults.h")
    if json_path is None:
        json_path = os.path.join(os.path.dirname(__file__), "line_calib.json")

    print("\n" + "*" * 65)
    print("  [ACTION REQUIRED] SWIPE SENSORS OVER WHITE PAPER & BLACK TAPE!")
    print(f"  Calibration duration: {duration_ms / 1000.0:.1f} seconds...")
    print("*" * 65 + "\n")

    stats = CalibrationStats(num_sensors=5)
    queue = asyncio.Queue()

    def on_line(line):
        parsed = parse_line_telemetry(line)
        if parsed:
            queue.put_nowait(parsed["raw"])

    client.add_line_listener(on_line)
    await client.send_command(f"LINE_CALIB {duration_ms}")

    start_time = time.time()
    last_update = 0

    try:
        while (time.time() - start_time) * 1000 < duration_ms + 1500:
            try:
                raw = await asyncio.wait_for(queue.get(), timeout=0.5)
                stats.update(raw)
                now = time.time()
                if now - last_update > 0.15:
                    last_update = now
                    elapsed = int((now - start_time) * 1000)
                    rem = max(0, duration_ms - elapsed)
                    sys.stdout.write(
                        f"\r[Calibrating... {elapsed:5d}ms / {duration_ms}ms | Remaining: {rem:5d}ms] Samples: {stats.sample_count:4d} | Raw: {raw}"
                    )
                    sys.stdout.flush()
            except asyncio.TimeoutError:
                continue

    except asyncio.CancelledError:
        pass
    finally:
        client.remove_line_listener(on_line)

    print("\n\n[+] Calibration collection finished! Samples collected:", stats.sample_count)
    if stats.sample_count < 10:
        print(colorize("[-] Insufficient telemetry samples received.", COLOR_RED))
        return False

    print("\n" + "=" * 68)
    print("           AGV STM32 - IR ARRAY CALIBRATION REPORT")
    print("=" * 68)
    print(f"{'Channel':<15} | {'Min (White)':>12} | {'Max (Black)':>12} | {'Delta':>8} | {'Status':<10}")
    print("-" * 68)

    is_valid, errors = validate_calibration(stats.min_raw, stats.max_raw, min_dynamic_range=100)
    for i in range(5):
        mn = stats.min_raw[i]
        mx = stats.max_raw[i]
        delta = mx - mn
        st = "PASS" if delta >= 100 and mn < mx else "FAIL"
        st_colored = colorize(st, COLOR_GREEN if st == "PASS" else COLOR_RED)
        print(f"{SENSOR_NAMES[i]:<15} | {mn:>12d} | {mx:>12d} | {delta:>8d} | {st_colored:<10}")

    print("=" * 68)
    if is_valid:
        print(colorize("  [SUCCESS] All 5 IR sensors passed dynamic range validation (>= 100)!", COLOR_GREEN))
    else:
        print(colorize("  [WARNING] One or more sensors failed validation:", COLOR_RED))
        for err in errors:
            print(f"    - {err}")
    print("=" * 68 + "\n")

    if save and is_valid:
        # Save JSON
        os.makedirs(os.path.dirname(os.path.abspath(json_path)), exist_ok=True)
        payload = {
            "timestamp": datetime.datetime.now().isoformat(),
            "num_sensors": 5,
            "min_raw": stats.min_raw,
            "max_raw": stats.max_raw,
            "delta": [stats.max_raw[i] - stats.min_raw[i] for i in range(5)],
        }
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"[+] Calibration JSON saved to: {json_path}")

        # Update C Header
        if os.path.exists(os.path.dirname(os.path.abspath(header_path))):
            header_content = generate_header_content(stats.min_raw, stats.max_raw)
            with open(header_path, "w", encoding="utf-8") as f:
                f.write(header_content)
            print(f"[+] STM32 C Header updated: {header_path}")
        else:
            print(f"[-] Directory for header not found: {header_path}, skipped header write.")

    return is_valid


async def run_step_test_workflow(
    client: AGVBleClient,
    motor: str = "L",
    pwm: int = 600,
    duration_ms: int = 1500,
    plot: bool = False,
    csv_path: str = "step_response_ble.csv",
):
    """
    Executes a step test, records telemetry to CSV, prints metrics, and optionally plots.
    """
    print(f"\n[+] Executing Step Test: Motor {motor}, PWM {pwm}, Duration {duration_ms} ms...")
    step_samples = []
    queue = asyncio.Queue()

    def on_line(line):
        parsed = parse_step_telemetry(line)
        if parsed:
            queue.put_nowait(parsed)
        elif line.startswith("[STM32 LOG]") or line.startswith("[RESP]"):
            print(f"  {line}")

    client.add_line_listener(on_line)
    await client.send_command(f"STEP_TEST {motor} {pwm} {duration_ms}")

    start_time = time.time()
    try:
        while (time.time() - start_time) * 1000 < duration_ms + 1500:
            try:
                sample = await asyncio.wait_for(queue.get(), timeout=0.5)
                step_samples.append(sample)
                sys.stdout.write(
                    f"\r[Step Running] Samples: {len(step_samples):3d} | Time: {sample['time_s']:.3f}s | Speed: {sample['rpm']:.1f} RPM"
                )
                sys.stdout.flush()
            except asyncio.TimeoutError:
                continue

    except asyncio.CancelledError:
        pass
    finally:
        client.remove_line_listener(on_line)

    print(f"\n\n[+] Step Test complete! Collected {len(step_samples)} telemetry data points.")
    if not step_samples:
        print(colorize("[-] No step telemetry received.", COLOR_RED))
        return

    # Save to CSV
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Time_ms", "Time_s", "PWM", "PWM_Pct", "DeltaTicks", "Speed_RPM", "Speed_RadS"])
        for s in step_samples:
            writer.writerow([s["elapsed_ms"], s["time_s"], s["pwm"], s["pwm_pct"], s["delta_ticks"], s["rpm"], s["rads"]])
    print(f"[+] Saved step response data to CSV: {csv_path}")

    # Compute key metrics
    rpms = [s["rpm"] for s in step_samples]
    peak_rpm = max(rpms) if rpms else 0.0
    # Steady state: average of last 25% of samples
    tail_len = max(1, len(rpms) // 4)
    ss_rpm = sum(rpms[-tail_len:]) / tail_len

    print("\n" + "=" * 50)
    print("       MOTOR STEP RESPONSE METRICS")
    print("=" * 50)
    print(f"  Motor:                 {motor}")
    print(f"  Step Amplitude (PWM):  {pwm} ({(pwm / 1000.0) * 100:.1f}%)")
    print(f"  Peak Speed:            {peak_rpm:.2f} RPM")
    print(f"  Steady-State Speed:    {ss_rpm:.2f} RPM")
    print("=" * 50 + "\n")

    if plot:
        try:
            import matplotlib.pyplot as plt

            times = [s["time_s"] for s in step_samples]
            pwms = [s["pwm_pct"] for s in step_samples]

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

            ax1.plot(times, rpms, "b-", linewidth=2, label="Measured RPM")
            ax1.axhline(ss_rpm, color="r", linestyle="--", label=f"Steady State: {ss_rpm:.1f} RPM")
            ax1.set_ylabel("Speed (RPM)")
            ax1.set_title(f"AGV Motor Step Response - Motor {motor} @ {pwm} PWM")
            ax1.grid(True, linestyle=":", alpha=0.6)
            ax1.legend()

            ax2.plot(times, pwms, "g-", linewidth=2, label="Control Effort (% PWM)")
            ax2.set_xlabel("Time (seconds)")
            ax2.set_ylabel("PWM (%)")
            ax2.grid(True, linestyle=":", alpha=0.6)
            ax2.legend()

            plt.tight_layout()
            print("[+] Displaying Matplotlib step response plot (close window to continue)...")
            plt.show()
        except ImportError:
            print(colorize("[-] matplotlib is not installed, cannot display plot.", COLOR_YELLOW))
        except Exception as e:
            print(f"[-] Plotting failed: {e}")


# =============================================================================
# Interactive REPL Loop
# =============================================================================

async def run_repl(client: AGVBleClient):
    """Interactive command-line shell with colorized log output."""
    print("\n" + "=" * 60)
    print("  AGV ESP32 WIRELESS BLE BENCH-TEST REPL")
    print("  Type 'help' for available commands, 'quit' or 'exit' to disconnect.")
    print("  Quick routines: 'monitor_ir [ms]', 'calibrate [ms]', 'step <L|R> <pwm> <ms> [--plot]'")
    print("=" * 60 + "\n")

    def print_log(line):
        if line.startswith("DATA:"):
            # Don't clutter REPL prompt with raw telemetry unless in diagnostic mode
            return

        if line.startswith("[RESP]"):
            print(f"\r{colorize(line, COLOR_GREEN)}")
        elif line.startswith("[ACK]"):
            print(f"\r{colorize(line, COLOR_CYAN)}")
        elif line.startswith("[ERR]"):
            print(f"\r{colorize(line, COLOR_RED)}")
        elif line.startswith("[STM32 LOG]"):
            print(f"\r{colorize(line, COLOR_YELLOW)}")
        elif line.startswith("[MISSION]"):
            print(f"\r{colorize(line, COLOR_MAGENTA)}")
        elif line.startswith("[ENCODER]"):
            print(f"\r{colorize(line, COLOR_CYAN)}")
        else:
            print(f"\r{line}")
        sys.stdout.write("agv> ")
        sys.stdout.flush()

    client.add_line_listener(print_log)

    loop = asyncio.get_running_loop()
    try:
        while True:
            # Read non-blocking input from user
            sys.stdout.write("agv> ")
            sys.stdout.flush()
            user_input = await loop.run_in_executor(None, sys.stdin.readline)
            if not user_input:
                break

            cmd_raw = user_input.strip()
            if not cmd_raw:
                continue

            if cmd_raw.lower() in ("exit", "quit", "q"):
                print("[+] Exiting REPL...")
                break

            # Check for high-level routines inside REPL
            tokens = cmd_raw.split()
            c0 = tokens[0].lower()

            if c0 in ("monitor_ir", "monitor"):
                dur = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else 15000
                await run_ir_monitor_workflow(client, duration_ms=dur)
            elif c0 in ("calibrate", "calib", "line_calib"):
                dur = int(tokens[1]) if len(tokens) > 1 and tokens[1].isdigit() else 10000
                await run_calibration_workflow(client, duration_ms=dur)
            elif c0 in ("step", "step_test"):
                motor = tokens[1].upper() if len(tokens) > 1 else "L"
                pwm = int(tokens[2]) if len(tokens) > 2 else 600
                dur = int(tokens[3]) if len(tokens) > 3 else 1500
                plot = "--plot" in tokens or "-p" in tokens
                await run_step_test_workflow(client, motor=motor, pwm=pwm, duration_ms=dur, plot=plot)
            else:
                translated = translate_cli_command(cmd_raw)
                await client.send_command(translated)

    finally:
        client.remove_line_listener(print_log)


# =============================================================================
# CLI Main
# =============================================================================

async def main_async():
    parser = argparse.ArgumentParser(description="AGV ESP32 Wireless BLE Diagnostics Workbench")
    parser.add_argument("--device", type=str, default=DEFAULT_DEVICE_NAME, help="BLE Device Name or MAC address")
    parser.add_argument("--scan", action="store_true", help="Scan and list nearby BLE devices, then exit")
    parser.add_argument("--monitor-ir", nargs="?", const=15000, type=int, help="Run 5 IR sensor live monitor (ms)")
    parser.add_argument("--calibrate", nargs="?", const=10000, type=int, help="Run IR desk calibration (ms)")
    parser.add_argument("--step", nargs=3, metavar=("MOTOR", "PWM", "MS"), help="Run step test (e.g. --step L 600 1500)")
    parser.add_argument("--plot", action="store_true", help="Plot step response with Matplotlib")
    parser.add_argument("--cmd", type=str, help="Send single command and exit")

    args = parser.parse_args()

    if args.scan:
        if BleakScanner is None:
            print("[-] bleak is not installed. Run 'pip install bleak'")
            return
        print("[+] Scanning for BLE devices (5 seconds)...")
        devices = await BleakScanner.discover(timeout=5.0)
        print(f"\nFound {len(devices)} devices:")
        for d in devices:
            print(f"  - {d.address} | Name: {d.name or '<Unknown>'}")
        return

    client = AGVBleClient(device_name=args.device)
    try:
        await client.connect(timeout=8.0)
    except Exception as e:
        print(f"[-] Connection failed: {e}")
        return

    try:
        if args.monitor_ir is not None:
            await run_ir_monitor_workflow(client, duration_ms=args.monitor_ir)
        elif args.calibrate is not None:
            await run_calibration_workflow(client, duration_ms=args.calibrate)
        elif args.step is not None:
            m = args.step[0].upper()
            p = int(args.step[1])
            d = int(args.step[2])
            await run_step_test_workflow(client, motor=m, pwm=p, duration_ms=d, plot=args.plot)
        elif args.cmd:
            translated = translate_cli_command(args.cmd)
            await client.send_command(translated)
            await asyncio.sleep(1.0)
        else:
            await run_repl(client)
    except KeyboardInterrupt:
        print("\n[!] User interrupted.")
    finally:
        await client.disconnect()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[!] Program terminated.")


if __name__ == "__main__":
    main()
