#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGV STM32 - IR Sensor Array Real-Time Monitor & Calibration Tool
Connects to OpenOCD RTT Server (localhost:9090) or Serial COM port,
triggers LINE_TEST diagnostic routines, and visualizes 5 IR sensor readings.
"""

import sys
import time
import socket
import argparse
import re

LINE_DATA_PATTERN = re.compile(
    r"^DATA:LINE,(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(-?\d+),(\d+)"
)


def render_bar(val, max_val=1000, width=20):
    clamped = max(0, min(val, max_val))
    fill = int((clamped / max_val) * width)
    return "█" * fill + "░" * (width - fill)


def run_monitor(host="localhost", port=9090, mode="SENSOR", duration=10000, base_rpm=80):
    print(f"[+] Connecting to RTT Server at {host}:{port}...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3.0)
        s.connect((host, port))
        print("[+] Connected successfully!")
    except Exception as e:
        print(f"[-] Connection failed: {e}")
        print("    -> Make sure OpenOCD RTT Server is running: ./rtt.sh")
        return

    # Trigger diagnostic command
    cmd = f"LINE_TEST {mode} {duration} {base_rpm}\n"
    print(f"[+] Sending diagnostic command: {cmd.strip()}")
    s.sendall(cmd.encode("utf-8"))

    buffer = ""
    start_time = time.time()

    print("\n--- LIVE IR SENSOR TELEMETRY (Ctrl+C to stop) ---")
    try:
        while True:
            try:
                data = s.recv(1024).decode("utf-8", errors="ignore")
            except socket.timeout:
                continue

            if not data:
                break

            buffer += data
            lines = buffer.split("\r\n")
            buffer = lines.pop()

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                m = LINE_DATA_PATTERN.match(line)
                if m:
                    elapsed_ms = int(m.group(1))
                    raw = [int(m.group(i)) for i in range(2, 7)]
                    norm = [int(m.group(i)) for i in range(7, 12)]
                    err = int(m.group(12))
                    line_seen = int(m.group(13)) == 1

                    # Clear screen / carriage return visualization
                    sys.stdout.write("\033[H\033[J")
                    print(f"==================================================")
                    print(f"  AGV STM32 IR ARRAY DIAGNOSTICS - MODE: {mode}")
                    print(f"  Elapsed: {elapsed_ms} ms | Line Detected: {'YES [ON TRACK]' if line_seen else 'NO [LOST]'}")
                    print(f"  Centroid Error (e_line): {err:>4d} (Target: 0)")
                    print(f"==================================================")
                    sensor_names = ["IR_R2 (Ch0)", "IR_R1 (Ch1)", "IR_MID(Ch2)", "IR_L1 (Ch3)", "IR_L2 (Ch4)"]
                    for idx in range(5):
                        bar = render_bar(norm[idx], max_val=1000, width=25)
                        print(f"  {sensor_names[idx]} |{bar}| Norm:{norm[idx]:>4d} | Raw ADC:{raw[idx]:>4d}")
                    print(f"==================================================")
                    print(f"  Press Ctrl+C or send 'LINE_TEST STOP' to finish.")
                    sys.stdout.flush()
                elif "DATA_END" in line or "[DIAG_IR] Completed" in line:
                    print(f"\n[+] Diagnostic test completed:\n{line}")
                elif "[DIAG_IR]" in line or "Ack:" in line:
                    print(f"  [STM32] {line}")

    except KeyboardInterrupt:
        print("\n[!] Stopping diagnostic test...")
        try:
            s.sendall(b"LINE_TEST STOP\n")
            time.sleep(0.2)
        except Exception:
            pass
    finally:
        s.close()
        print("[+] Session closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AGV STM32 IR Diagnostics Monitor")
    parser.add_argument("--host", default="localhost", help="RTT Server Host (default: localhost)")
    parser.add_argument("--port", type=int, default=9090, help="RTT Port (default: 9090)")
    parser.add_argument("--mode", default="SENSOR", choices=["SENSOR", "CALIB", "RUN"], help="Diagnostic mode")
    parser.add_argument("--duration", type=int, default=15000, help="Test duration in ms (default: 15000)")
    parser.add_argument("--rpm", type=float, default=80.0, help="Base RPM for RUN mode (default: 80)")
    args = parser.parse_args()

    run_monitor(args.host, args.port, args.mode, args.duration, args.rpm)

