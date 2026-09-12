#!/usr/bin/env python3
"""
AGV Wireless Desktop GUI Dashboard
Built with CustomTkinter, Bleak (BLE 5.0), and Matplotlib.
Provides real-time telemetry visualization, manual jog control,
step response testing, optical calibration, and autonomous route planning.
"""

import sys
import os
import time
import re
import queue
import threading
import asyncio
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Tuple

# Third-party imports
try:
    import customtkinter as ctk
except ImportError:
    ctk = None

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    BleakScanner = None
    BleakClient = None

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError:
    Figure = None
    FigureCanvasTkAgg = None

# BLE Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Client writes to ESP32
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # ESP32 notifies Client


# =============================================================================
# 1. Telemetry Data Models & Parser
# =============================================================================

@dataclass
class LineTelemetry:
    timestamp_ms: int
    raw_values: List[int]
    norm_values: List[int]
    line_error: float
    line_seen: bool


@dataclass
class StepPoint:
    time_ms: float
    time_s: float
    pwm: int
    pct: float
    ticks: int
    speed_rpm: float
    speed_rads: float


class TelemetryParser:
    """Pure logic parser to decompose incoming BLE strings into structured events."""

    def __init__(self):
        pass

    def parse_line(self, line: str) -> Optional[Dict[str, Any]]:
        if not line:
            return None
        line = line.strip()
        if not line:
            return None

        # 1. Check for Line Sensor Telemetry (DATA:LINE,...)
        # Format: DATA:LINE,<ms>,<raw0..4>,<norm0..4>,<err>,<line_seen> (14 tokens total)
        if line.startswith("DATA:LINE,"):
            tokens = line[10:].split(",")
            if len(tokens) >= 13:
                try:
                    ts_ms = int(tokens[0])
                    raws = [int(x) for x in tokens[1:6]]
                    norms = [int(x) for x in tokens[6:11]]
                    err = float(tokens[11])
                    seen = bool(int(tokens[12]))
                    return {
                        "type": "LINE",
                        "data": LineTelemetry(ts_ms, raws, norms, err, seen),
                        "raw_str": line
                    }
                except (ValueError, IndexError):
                    pass

        # 2. Check for Motor Step Response Data (DATA:<ms>,...)
        # Format: DATA:<ms>,<time_s>,<pwm>,<pct>,<ticks>,<rpm>,<rads> (7 tokens total)
        elif line.startswith("DATA:"):
            tokens = line[5:].split(",")
            if len(tokens) >= 7:
                try:
                    t_ms = float(tokens[0])
                    t_s = float(tokens[1])
                    pwm = int(tokens[2])
                    pct = float(tokens[3])
                    ticks = int(tokens[4])
                    rpm = float(tokens[5])
                    rads = float(tokens[6])
                    return {
                        "type": "STEP",
                        "data": StepPoint(t_ms, t_s, pwm, pct, ticks, rpm, rads),
                        "raw_str": line
                    }
                except (ValueError, IndexError):
                    pass

        # 3. Check for Encoder response
        # Format: [ENCODER] Left: <l> | Right: <r>
        if "[ENCODER]" in line:
            match = re.search(r"Left:\s*(-?\d+)\s*\|\s*Right:\s*(-?\d+)", line)
            if match:
                return {
                    "type": "ENCODER",
                    "left": int(match.group(1)),
                    "right": int(match.group(2)),
                    "raw_str": line
                }

        # 4. Check for Mission Status
        # Format: [MISSION] State: <state>, Step: <step>
        if "[MISSION]" in line:
            match = re.search(r"State:\s*([A-Z_]+),\s*Step:\s*(\d+)", line)
            if match:
                return {
                    "type": "MISSION",
                    "state": match.group(1),
                    "step": int(match.group(2)),
                    "raw_str": line
                }

        # 5. Check for [RESP] messages
        if line.startswith("[RESP]"):
            return {
                "type": "RESP",
                "text": line[6:].strip(),
                "raw_str": line
            }

        # 6. Check for [ACK] messages
        if line.startswith("[ACK]"):
            return {
                "type": "ACK",
                "text": line[5:].strip(),
                "raw_str": line
            }

        # 7. Check for [ERR] messages
        if line.startswith("[ERR]"):
            return {
                "type": "ERR",
                "text": line[5:].strip(),
                "raw_str": line
            }

        # 8. Check for [STM32 LOG]
        if line.startswith("[STM32 LOG]"):
            return {
                "type": "LOG",
                "text": line[11:].strip(),
                "raw_str": line
            }

        # Fallback to general message
        return {
            "type": "GENERAL",
            "text": line,
            "raw_str": line
        }


# =============================================================================
# 2. Asynchronous BLE Worker (Thread-Safe with Queue)
# =============================================================================

class AsyncBleWorker:
    """Manages Bleak asyncio operations in a background thread."""

    def __init__(self, tx_queue: queue.Queue, rx_queue: queue.Queue):
        self.tx_queue = tx_queue
        self.rx_queue = rx_queue
        self.parser = TelemetryParser()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.client: Optional[BleakClient] = None
        self.is_running = False
        self.target_device_name = "AGV_ESP32"

    def start(self):
        self.is_running = True
        thread = threading.Thread(target=self._run_loop, daemon=True, name="BleWorkerThread")
        thread.start()

    def _run_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main_async_task())

    async def _main_async_task(self):
        while self.is_running:
            # Check commands from GUI
            try:
                msg = self.tx_queue.get_nowait()
                action = msg.get("action")
                if action == "connect":
                    target_name = msg.get("target_name", "AGV_ESP32")
                    await self._do_connect(target_name)
                elif action == "disconnect":
                    await self._do_disconnect()
                elif action == "send":
                    cmd = msg.get("cmd", "")
                    await self._do_send(cmd)
            except queue.Empty:
                pass

            await asyncio.sleep(0.01)

    async def _do_connect(self, target_name: str):
        self.rx_queue.put({"event": "status", "status": "SCANNING", "msg": f"Scanning for {target_name}..."})
        try:
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name and target_name.lower() in d.name.lower()),
                timeout=5.0
            )
            if not device:
                self.rx_queue.put({"event": "status", "status": "DISCONNECTED", "msg": f"Device '{target_name}' not found"})
                return

            self.rx_queue.put({"event": "status", "status": "CONNECTING", "msg": f"Connecting to {device.address}..."})
            self.client = BleakClient(device, disconnected_callback=self._on_disconnected)
            await self.client.connect()

            # Start notification on NUS TX characteristic
            await self.client.start_notify(NUS_TX_CHAR_UUID, self._on_rx_notification)
            mtu = getattr(self.client, "mtu_size", 256)

            self.rx_queue.put({
                "event": "status",
                "status": "CONNECTED",
                "address": device.address,
                "name": device.name,
                "mtu": mtu,
                "msg": f"Connected to {device.name} ({device.address})"
            })
        except Exception as e:
            self.rx_queue.put({"event": "status", "status": "DISCONNECTED", "msg": f"Connect failed: {e}"})

    async def _do_disconnect(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self.client = None
        self.rx_queue.put({"event": "status", "status": "DISCONNECTED", "msg": "Disconnected"})

    async def _do_send(self, cmd: str):
        if not self.client or not self.client.is_connected:
            self.rx_queue.put({"event": "error", "msg": "Cannot send: Not connected to BLE"})
            return

        data_bytes = (cmd.strip() + "\r\n").encode("utf-8")
        try:
            await self.client.write_gatt_char(NUS_RX_CHAR_UUID, data_bytes, response=False)
            self.rx_queue.put({"event": "tx", "cmd": cmd.strip()})
        except Exception as e:
            self.rx_queue.put({"event": "error", "msg": f"Send failed: {e}"})

    def _on_rx_notification(self, sender: int, data: bytearray):
        text = data.decode("utf-8", errors="replace")
        for line in text.splitlines():
            line = line.strip()
            if line:
                parsed = self.parser.parse_line(line)
                if parsed:
                    self.rx_queue.put({"event": "telemetry", "parsed": parsed, "raw": line})

    def _on_disconnected(self, client: BleakClient):
        self.rx_queue.put({"event": "status", "status": "DISCONNECTED", "msg": "Device disconnected"})


# =============================================================================
# 3. CustomTkinter GUI Dashboard
# =============================================================================

class AgvDashboardApp:
    """Modern Desktop GUI for AGV Robot Control and Telemetry."""

    def __init__(self):
        if ctk is None:
            raise RuntimeError("customtkinter is required. Run 'pip install customtkinter'")

        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.root = ctk.CTk()
        self.root.title("AGV Wireless Diagnostics & Control Dashboard")
        self.root.geometry("1180x820")
        self.root.minsize(1000, 720)

        # Queues for background BLE thread
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()
        self.ble_worker = AsyncBleWorker(self.tx_queue, self.rx_queue)
        self.ble_worker.start()

        self.is_connected = False
        self.current_speed_rpm = 50.0

        # Step response buffer
        self.step_data_x: List[float] = []
        self.step_data_y: List[float] = []
        self.is_recording_step = False

        # Calibration buffer
        self.calib_min = [4095] * 5
        self.calib_max = [0] * 5
        self.is_calibrating = False

        # Route planner actions
        self.route_actions: List[str] = []

        self._create_widgets()

        # Start periodic GUI queue polling (20ms loop)
        self.root.after(20, self._poll_ble_events)

    def _create_widgets(self):
        # ---------------------------------------------------------------------
        # Top Header Bar: Title, Connection Status, Emergency Stop
        # ---------------------------------------------------------------------
        self.header_frame = ctk.CTkFrame(self.root, height=65, corner_radius=10)
        self.header_frame.pack(fill="x", padx=15, pady=(12, 6))

        # Title and Badge
        self.lbl_title = ctk.CTkLabel(
            self.header_frame,
            text="🚗 AGV ROBOT MASTER",
            font=ctk.CTkFont(size=20, weight="bold")
        )
        self.lbl_title.pack(side="left", padx=15)

        self.lbl_status = ctk.CTkLabel(
            self.header_frame,
            text="🔴 DISCONNECTED",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color="#FF5252"
        )
        self.lbl_status.pack(side="left", padx=10)

        # Connect / Disconnect Buttons
        self.entry_device = ctk.CTkEntry(self.header_frame, width=130, placeholder_text="AGV_ESP32")
        self.entry_device.insert(0, "AGV_ESP32")
        self.entry_device.pack(side="left", padx=(15, 5))

        self.btn_connect = ctk.CTkButton(
            self.header_frame,
            text="Scan & Connect",
            width=110,
            command=self._toggle_connection
        )
        self.btn_connect.pack(side="left", padx=5)

        self.btn_ping = ctk.CTkButton(
            self.header_frame,
            text="Ping",
            width=65,
            fg_color="#37474F",
            command=lambda: self._send_cmd("PING")
        )
        self.btn_ping.pack(side="left", padx=5)

        # Big Red Emergency Stop Button
        self.btn_emg = ctk.CTkButton(
            self.header_frame,
            text="🛑 EMERGENCY STOP",
            font=ctk.CTkFont(size=15, weight="bold"),
            fg_color="#D32F2F",
            hover_color="#B71C1C",
            width=190,
            height=40,
            command=self._emergency_stop
        )
        self.btn_emg.pack(side="right", padx=15)

        # ---------------------------------------------------------------------
        # Center Tabview: 4 Dedicated Diagnostics Tabs
        # ---------------------------------------------------------------------
        self.tabview = ctk.CTkTabview(self.root, corner_radius=10)
        self.tabview.pack(fill="both", expand=True, padx=15, pady=6)

        self.tab_dashboard = self.tabview.add("Dashboard & Sensors")
        self.tab_step = self.tabview.add("Motor Step Response")
        self.tab_calib = self.tabview.add("Line Optical Calib")
        self.tab_route = self.tabview.add("Mission Route Planner")

        self._build_tab_dashboard()
        self._build_tab_step()
        self._build_tab_calib()
        self._build_tab_route()

        # ---------------------------------------------------------------------
        # Bottom Console Log Frame
        # ---------------------------------------------------------------------
        self.console_frame = ctk.CTkFrame(self.root, height=160, corner_radius=10)
        self.console_frame.pack(fill="x", padx=15, pady=(6, 12))

        # Console Header
        console_bar = ctk.CTkFrame(self.console_frame, fg_color="transparent")
        console_bar.pack(fill="x", padx=10, pady=(6, 2))

        lbl_console = ctk.CTkLabel(console_bar, text="Terminal Log & Raw Command Input", font=ctk.CTkFont(size=12, weight="bold"))
        lbl_console.pack(side="left")

        btn_clear = ctk.CTkButton(console_bar, text="Clear Log", width=70, height=22, font=ctk.CTkFont(size=11), command=self._clear_log)
        btn_clear.pack(side="right", padx=5)

        # Textbox Console
        self.txt_console = ctk.CTkTextbox(self.console_frame, height=90, font=ctk.CTkFont(family="Consolas", size=11))
        self.txt_console.pack(fill="both", expand=True, padx=10, pady=(2, 6))

        # CLI Input Bar
        input_bar = ctk.CTkFrame(self.console_frame, fg_color="transparent")
        input_bar.pack(fill="x", padx=10, pady=(0, 6))

        self.entry_cmd = ctk.CTkEntry(input_bar, placeholder_text="Enter CLI command (e.g. HELP, GET_ENC, STEP_TEST L 500 2000)...")
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.entry_cmd.bind("<Return>", lambda e: self._send_custom_command())

        self.btn_send_cmd = ctk.CTkButton(input_bar, text="Send", width=80, command=self._send_custom_command)
        self.btn_send_cmd.pack(side="right")

    # =========================================================================
    # Tab 1: Dashboard & Live Sensors
    # =========================================================================
    def _build_tab_dashboard(self):
        frame = self.tab_dashboard

        # 2-Column layout
        col_left = ctk.CTkFrame(frame, corner_radius=8)
        col_left.pack(side="left", fill="both", expand=True, padx=(5, 5), pady=5)

        col_right = ctk.CTkFrame(frame, corner_radius=8)
        col_right.pack(side="right", fill="both", expand=True, padx=(5, 5), pady=5)

        # Left Column: 5 IR Sensor Bars
        ctk.CTkLabel(col_left, text="📡 5-CHANNEL ANALOG IR SENSOR ARRAY", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(10, 5))

        self.ir_bars: List[ctk.CTkProgressBar] = []
        self.ir_labels: List[ctk.CTkLabel] = []

        ir_container = ctk.CTkFrame(col_left, fg_color="transparent")
        ir_container.pack(fill="x", padx=20, pady=5)

        sensor_names = ["IR0 (Far Left)", "IR1 (Left)", "IR2 (Center)", "IR3 (Right)", "IR4 (Far Right)"]
        for i in range(5):
            row = ctk.CTkFrame(ir_container, fg_color="transparent")
            row.pack(fill="x", pady=4)

            lbl = ctk.CTkLabel(row, text=f"{sensor_names[i]}: ---", width=160, anchor="w", font=ctk.CTkFont(size=12))
            lbl.pack(side="left")
            self.ir_labels.append(lbl)

            bar = ctk.CTkProgressBar(row, orientation="horizontal", height=14)
            bar.set(0.0)
            bar.pack(side="right", fill="x", expand=True, padx=(10, 0))
            self.ir_bars.append(bar)

        # Line Tracking Status indicators
        status_box = ctk.CTkFrame(col_left, corner_radius=6)
        status_box.pack(fill="x", padx=20, pady=10)

        self.lbl_line_detect = ctk.CTkLabel(status_box, text="Line Status: ⚪ Searching...", font=ctk.CTkFont(size=13, weight="bold"))
        self.lbl_line_detect.pack(side="left", padx=15, pady=8)

        self.lbl_line_err = ctk.CTkLabel(status_box, text="Error (e): 0.0 mm", font=ctk.CTkFont(size=13, weight="bold"))
        self.lbl_line_err.pack(side="right", padx=15, pady=8)

        # Action Buttons for IR
        ir_actions = ctk.CTkFrame(col_left, fg_color="transparent")
        ir_actions.pack(fill="x", padx=20, pady=5)

        self.btn_monitor_ir = ctk.CTkButton(ir_actions, text="Start 20Hz IR Monitor", command=lambda: self._send_cmd("MONITOR_IR 30000"))
        self.btn_monitor_ir.pack(side="left", fill="x", expand=True, padx=(0, 5))

        self.btn_stop_monitor = ctk.CTkButton(ir_actions, text="Stop Stream", fg_color="#455A64", command=lambda: self._send_cmd("STOP"))
        self.btn_stop_monitor.pack(side="right", fill="x", expand=True, padx=(5, 0))

        # Right Column: Manual Jog D-Pad & Speed
        ctk.CTkLabel(col_right, text="🕹️ MANUAL JOG & SPEED CONTROLS", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(10, 5))

        # Speed Slider
        speed_box = ctk.CTkFrame(col_right, fg_color="transparent")
        speed_box.pack(fill="x", padx=20, pady=5)

        self.lbl_speed = ctk.CTkLabel(speed_box, text="Target Speed: 50.0 RPM", font=ctk.CTkFont(size=12))
        self.lbl_speed.pack(side="left")

        self.slider_speed = ctk.CTkSlider(speed_box, from_=10, to=120, number_of_steps=22, command=self._on_speed_slider)
        self.slider_speed.set(50.0)
        self.slider_speed.pack(side="right", fill="x", expand=True, padx=(10, 0))

        # D-Pad Button Grid
        dpad_frame = ctk.CTkFrame(col_right, fg_color="transparent")
        dpad_frame.pack(pady=10)

        btn_fwd = ctk.CTkButton(dpad_frame, text="▲\nFORWARD (W)", width=120, height=45, command=self._jog_forward)
        btn_fwd.grid(row=0, column=1, padx=5, pady=5)

        btn_left = ctk.CTkButton(dpad_frame, text="◄\nLEFT (A)", width=120, height=45, command=self._jog_left)
        btn_left.grid(row=1, column=0, padx=5, pady=5)

        btn_stop = ctk.CTkButton(dpad_frame, text="■\nSTOP (Space)", width=120, height=45, fg_color="#C62828", hover_color="#B71C1C", command=self._jog_stop)
        btn_stop.grid(row=1, column=1, padx=5, pady=5)

        btn_right = ctk.CTkButton(dpad_frame, text="►\nRIGHT (D)", width=120, height=45, command=self._jog_right)
        btn_right.grid(row=1, column=2, padx=5, pady=5)

        btn_rev = ctk.CTkButton(dpad_frame, text="▼\nREVERSE (S)", width=120, height=45, command=self._jog_reverse)
        btn_rev.grid(row=2, column=1, padx=5, pady=5)

        # Encoder readout box
        enc_box = ctk.CTkFrame(col_right, corner_radius=6)
        enc_box.pack(fill="x", padx=20, pady=8)

        self.lbl_enc = ctk.CTkLabel(enc_box, text="Encoder: Left: --- | Right: ---", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_enc.pack(side="left", padx=15, pady=6)

        btn_get_enc = ctk.CTkButton(enc_box, text="Read", width=60, height=24, command=lambda: self._send_cmd("GET_ENC"))
        btn_get_enc.pack(side="right", padx=5)

        btn_reset_enc = ctk.CTkButton(enc_box, text="Reset", width=60, height=24, fg_color="#546E7A", command=lambda: self._send_cmd("RESET_ENC"))
        btn_reset_enc.pack(side="right", padx=5)

    # =========================================================================
    # Tab 2: Motor Step Response (Embedded Matplotlib Plot)
    # =========================================================================
    def _build_tab_step(self):
        frame = self.tab_step

        # Controls Left
        ctrl_frame = ctk.CTkFrame(frame, width=280, corner_radius=8)
        ctrl_frame.pack(side="left", fill="y", padx=(5, 5), pady=5)

        ctk.CTkLabel(ctrl_frame, text="STEP TEST CONTROLS", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(12, 8))

        # Motor choice
        self.step_motor_var = ctk.StringVar(value="L")
        r_box = ctk.CTkFrame(ctrl_frame, fg_color="transparent")
        r_box.pack(fill="x", padx=15, pady=4)
        ctk.CTkLabel(r_box, text="Target Motor:").pack(side="left")
        ctk.CTkRadioButton(r_box, text="Left", variable=self.step_motor_var, value="L").pack(side="left", padx=8)
        ctk.CTkRadioButton(r_box, text="Right", variable=self.step_motor_var, value="R").pack(side="left", padx=8)

        # PWM Slider
        ctk.CTkLabel(ctrl_frame, text="PWM Level (-1000..1000):", font=ctk.CTkFont(size=12)).pack(anchor="w", padx=15, pady=(8, 2))
        self.step_pwm_lbl = ctk.CTkLabel(ctrl_frame, text="PWM: 500 (50%)", font=ctk.CTkFont(size=11, weight="bold"))
        self.step_pwm_lbl.pack(anchor="w", padx=15)
        self.step_pwm_slider = ctk.CTkSlider(ctrl_frame, from_=-1000, to=1000, number_of_steps=40, command=self._on_pwm_slider)
        self.step_pwm_slider.set(500)
        self.step_pwm_slider.pack(fill="x", padx=15, pady=(2, 8))

        # Duration Slider
        ctk.CTkLabel(ctrl_frame, text="Duration (ms):", font=ctk.CTkFont(size=12)).pack(anchor="w", padx=15, pady=(8, 2))
        self.step_dur_lbl = ctk.CTkLabel(ctrl_frame, text="2000 ms", font=ctk.CTkFont(size=11, weight="bold"))
        self.step_dur_lbl.pack(anchor="w", padx=15)
        self.step_dur_slider = ctk.CTkSlider(ctrl_frame, from_=500, to=5000, number_of_steps=45, command=self._on_dur_slider)
        self.step_dur_slider.set(2000)
        self.step_dur_slider.pack(fill="x", padx=15, pady=(2, 8))

        # Buttons
        self.btn_run_step = ctk.CTkButton(ctrl_frame, text="▶ RUN STEP TEST", height=38, fg_color="#2E7D32", hover_color="#1B5E20", command=self._run_step_test)
        self.btn_run_step.pack(fill="x", padx=15, pady=12)

        self.btn_export_csv = ctk.CTkButton(ctrl_frame, text="Export CSV", fg_color="#37474F", command=self._export_step_csv)
        self.btn_export_csv.pack(fill="x", padx=15, pady=4)

        # Performance Metrics Box
        m_box = ctk.CTkFrame(ctrl_frame, corner_radius=6)
        m_box.pack(fill="x", padx=15, pady=12)
        self.lbl_step_metrics = ctk.CTkLabel(
            m_box,
            text="Metrics:\n• Peak: --- RPM\n• Steady: --- RPM\n• Rise Time: --- ms",
            justify="left",
            font=ctk.CTkFont(size=12)
        )
        self.lbl_step_metrics.pack(padx=10, pady=8, anchor="w")

        # Plot Right
        plot_frame = ctk.CTkFrame(frame, corner_radius=8)
        plot_frame.pack(side="right", fill="both", expand=True, padx=(5, 5), pady=5)

        if Figure and FigureCanvasTkAgg:
            self.fig = Figure(figsize=(6, 4), dpi=100, facecolor="#242424")
            self.ax = self.fig.add_subplot(111)
            self.ax.set_facecolor("#1E1E1E")
            self.ax.set_title("Step Response Characteristic: Time vs RPM", color="white", fontsize=12)
            self.ax.set_xlabel("Time (ms)", color="white", fontsize=10)
            self.ax.set_ylabel("Speed (RPM)", color="white", fontsize=10)
            self.ax.tick_params(colors="white")
            self.ax.grid(True, linestyle="--", alpha=0.3, color="cyan")
            self.line_plot, = self.ax.plot([], [], color="#00E676", linewidth=2.0, label="Speed (RPM)")
            self.ax.legend(facecolor="#2A2A2A", labelcolor="white")

            self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)
        else:
            ctk.CTkLabel(plot_frame, text="Matplotlib not available").pack(expand=True)

    def _on_pwm_slider(self, val):
        val_int = int(val)
        self.step_pwm_lbl.configure(text=f"PWM: {val_int} ({val_int/10:.1f}%)")

    def _on_dur_slider(self, val):
        self.step_dur_lbl.configure(text=f"{int(val)} ms")

    def _run_step_test(self):
        motor = self.step_motor_var.get()
        pwm = int(self.step_pwm_slider.get())
        ms = int(self.step_dur_slider.get())

        self.step_data_x.clear()
        self.step_data_y.clear()
        self.is_recording_step = True
        self._update_plot([], [])

        cmd = f"STEP_TEST {motor} {pwm} {ms}"
        self._send_cmd(cmd)
        self._append_log(f"[STEP TEST] Initiated on Motor {motor}, PWM={pwm}, Duration={ms}ms")

    def _update_plot(self, x_data: List[float], y_data: List[float]):
        if not hasattr(self, "ax"):
            return
        self.line_plot.set_data(x_data, y_data)
        if x_data and y_data:
            self.ax.set_xlim(0, max(max(x_data), 100))
            y_min = min(min(y_data), 0)
            y_max = max(max(y_data), 10)
            self.ax.set_ylim(y_min * 1.1 if y_min < 0 else 0, y_max * 1.15)
        self.canvas.draw_idle()

    def _export_step_csv(self):
        if not self.step_data_x:
            self._append_log("[EXPORT] No step data available to export.")
            return
        filename = "step_response_gui.csv"
        with open(filename, "w", encoding="utf-8") as f:
            f.write("time_ms,speed_rpm\n")
            for x, y in zip(self.step_data_x, self.step_data_y):
                f.write(f"{x:.1f},{y:.2f}\n")
        self._append_log(f"[EXPORT] Saved {len(self.step_data_x)} points to {filename}")

    # =========================================================================
    # Tab 3: Line Optical Calibration
    # =========================================================================
    def _build_tab_calib(self):
        frame = self.tab_calib

        card = ctk.CTkFrame(frame, corner_radius=8)
        card.pack(fill="both", expand=True, padx=20, pady=15)

        ctk.CTkLabel(card, text="🎯 OPTICAL LINE CALIBRATION WIZARD", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(15, 8))

        guide_text = (
            "1. Đặt mảng 5 mắt cảm biến lên bề mặt bàn test.\n"
            "2. Bấm 'Start 10s Calibration' rồi lia xe qua lại trên nền trắng và vạch đen.\n"
            "3. Phần mềm sẽ tự ghi nhận Min (nền sáng) và Max (vạch đen) của từng cảm biến.\n"
            "4. Đảm bảo dải động Δ = (Max - Min) >= 100 để thuật toán Centroid hoạt động chuẩn xác."
        )
        ctk.CTkLabel(card, text=guide_text, justify="left", font=ctk.CTkFont(size=12)).pack(padx=20, pady=8)

        self.btn_calib = ctk.CTkButton(
            card,
            text="▶ START 10s CALIBRATION",
            height=38,
            font=ctk.CTkFont(size=13, weight="bold"),
            command=self._start_calibration
        )
        self.btn_calib.pack(pady=10)

        # Table Min/Max
        table_frame = ctk.CTkFrame(card, fg_color="#1E1E1E", corner_radius=6)
        table_frame.pack(fill="x", padx=30, pady=10)

        headers = ["Sensor", "Min (White)", "Max (Black)", "Delta (Δ)", "Status"]
        for c, h in enumerate(headers):
            ctk.CTkLabel(table_frame, text=h, font=ctk.CTkFont(size=12, weight="bold")).grid(row=0, column=c, padx=15, pady=6)

        self.calib_rows = []
        for i in range(5):
            r_labels = []
            lbl_name = ctk.CTkLabel(table_frame, text=f"IR{i}")
            lbl_name.grid(row=i+1, column=0, padx=15, pady=4)

            lbl_min = ctk.CTkLabel(table_frame, text="---")
            lbl_min.grid(row=i+1, column=1, padx=15, pady=4)
            r_labels.append(lbl_min)

            lbl_max = ctk.CTkLabel(table_frame, text="---")
            lbl_max.grid(row=i+1, column=2, padx=15, pady=4)
            r_labels.append(lbl_max)

            lbl_delta = ctk.CTkLabel(table_frame, text="---")
            lbl_delta.grid(row=i+1, column=3, padx=15, pady=4)
            r_labels.append(lbl_delta)

            lbl_st = ctk.CTkLabel(table_frame, text="Pending", text_color="#FFA726")
            lbl_st.grid(row=i+1, column=4, padx=15, pady=4)
            r_labels.append(lbl_st)

            self.calib_rows.append(r_labels)

        self.btn_save_calib = ctk.CTkButton(card, text="Save Calibration C Header", fg_color="#37474F", command=self._save_calib_header)
        self.btn_save_calib.pack(pady=10)

    def _start_calibration(self):
        self.calib_min = [4095] * 5
        self.calib_max = [0] * 5
        self.is_calibrating = True
        self._send_cmd("LINE_CALIB 10000")
        self._append_log("[CALIB] Calibration started for 10 seconds. Move sensors across white/black surfaces!")

    def _save_calib_header(self):
        header_path = "app_line_calib_defaults.h"
        content = (
            "#ifndef APP_LINE_CALIB_DEFAULTS_H\n"
            "#define APP_LINE_CALIB_DEFAULTS_H\n\n"
            "/* Auto-generated by AGV BLE Dashboard */\n"
            f"#define APP_LINE_CALIB_MIN_INIT  {{ {', '.join(str(x) for x in self.calib_min)} }}\n"
            f"#define APP_LINE_CALIB_MAX_INIT  {{ {', '.join(str(x) for x in self.calib_max)} }}\n\n"
            "#endif /* APP_LINE_CALIB_DEFAULTS_H */\n"
        )
        with open(header_path, "w", encoding="utf-8") as f:
            f.write(content)
        self._append_log(f"[CALIB] Calibration header saved to {header_path}")

    # =========================================================================
    # Tab 4: Route Mission Planner
    # =========================================================================
    def _build_tab_route(self):
        frame = self.tab_route

        card = ctk.CTkFrame(frame, corner_radius=8)
        card.pack(fill="both", expand=True, padx=20, pady=15)

        ctk.CTkLabel(card, text="🗺️ AUTONOMOUS ROUTE MISSION BUILDER", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(15, 8))

        # Speed and action buttons
        cfg_bar = ctk.CTkFrame(card, fg_color="transparent")
        cfg_bar.pack(fill="x", padx=20, pady=5)

        ctk.CTkLabel(cfg_bar, text="Base RPM:").pack(side="left")
        self.entry_route_rpm = ctk.CTkEntry(cfg_bar, width=70)
        self.entry_route_rpm.insert(0, "60")
        self.entry_route_rpm.pack(side="left", padx=8)

        # Action sequence builder buttons
        btn_bar = ctk.CTkFrame(card, fg_color="transparent")
        btn_bar.pack(fill="x", padx=20, pady=10)

        ctk.CTkButton(btn_bar, text="+ Forward (F)", width=100, command=lambda: self._add_route_action("F")).pack(side="left", padx=4)
        ctk.CTkButton(btn_bar, text="+ Turn Left (L)", width=100, command=lambda: self._add_route_action("L")).pack(side="left", padx=4)
        ctk.CTkButton(btn_bar, text="+ Turn Right (R)", width=100, command=lambda: self._add_route_action("R")).pack(side="left", padx=4)
        ctk.CTkButton(btn_bar, text="+ Stop (S)", width=80, fg_color="#C62828", command=lambda: self._add_route_action("S")).pack(side="left", padx=4)
        ctk.CTkButton(btn_bar, text="Clear", width=70, fg_color="#455A64", command=self._clear_route_actions).pack(side="left", padx=10)

        # Display sequence
        self.lbl_route_seq = ctk.CTkLabel(
            card,
            text="Current Route Sequence: [EMPTY]",
            font=ctk.CTkFont(family="Consolas", size=13, weight="bold"),
            fg_color="#1E1E1E",
            corner_radius=6,
            height=40
        )
        self.lbl_route_seq.pack(fill="x", padx=20, pady=10)

        # Mission control buttons
        m_ctrl = ctk.CTkFrame(card, fg_color="transparent")
        m_ctrl.pack(fill="x", padx=20, pady=10)

        self.btn_load_route = ctk.CTkButton(m_ctrl, text="1. Load Route to STM32", command=self._load_route)
        self.btn_load_route.pack(side="left", fill="x", expand=True, padx=4)

        self.btn_start_route = ctk.CTkButton(m_ctrl, text="2. Start Mission", fg_color="#2E7D32", command=lambda: self._send_cmd("ROUTE_START"))
        self.btn_start_route.pack(side="left", fill="x", expand=True, padx=4)

        self.btn_cancel_route = ctk.CTkButton(m_ctrl, text="Cancel Mission", fg_color="#C62828", command=lambda: self._send_cmd("ROUTE_CANCEL"))
        self.btn_cancel_route.pack(side="left", fill="x", expand=True, padx=4)

        self.btn_demo_route = ctk.CTkButton(m_ctrl, text="Demo Route (F,R,F,S)", fg_color="#37474F", command=lambda: self._send_cmd("ROUTE_TEST"))
        self.btn_demo_route.pack(side="left", fill="x", expand=True, padx=4)

        # Mission status display card
        self.card_mission_status = ctk.CTkLabel(
            card,
            text="Mission Status: IDLE | Step: 0",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color="#90CAF9"
        )
        self.card_mission_status.pack(pady=15)

    def _add_route_action(self, act: str):
        if len(self.route_actions) < 16:
            self.route_actions.append(act)
            self._update_route_display()

    def _clear_route_actions(self):
        self.route_actions.clear()
        self._update_route_display()

    def _update_route_display(self):
        if not self.route_actions:
            self.lbl_route_seq.configure(text="Current Route Sequence: [EMPTY]")
        else:
            txt = " ➔ ".join(self.route_actions)
            self.lbl_route_seq.configure(text=f"Current Route Sequence: {txt} ({len(self.route_actions)} steps)")

    def _load_route(self):
        if not self.route_actions:
            self._append_log("[ROUTE] Cannot load empty route!")
            return
        rpm = self.entry_route_rpm.get().strip()
        acts = "".join(self.route_actions)
        cmd = f"ROUTE_LOAD {rpm} {acts}"
        self._send_cmd(cmd)

    # =========================================================================
    # Manual Jog Handlers
    # =========================================================================
    def _on_speed_slider(self, val):
        self.current_speed_rpm = float(val)
        self.lbl_speed.configure(text=f"Target Speed: {self.current_speed_rpm:.1f} RPM")

    def _jog_forward(self):
        rpm = self.current_speed_rpm
        self._send_cmd(f"SET_RPM {rpm:.1f} {rpm:.1f}")

    def _jog_reverse(self):
        rpm = self.current_speed_rpm
        self._send_cmd(f"SET_RPM {-rpm:.1f} {-rpm:.1f}")

    def _jog_left(self):
        rpm = self.current_speed_rpm * 0.7
        self._send_cmd(f"SET_RPM {-rpm:.1f} {rpm:.1f}")

    def _jog_right(self):
        rpm = self.current_speed_rpm * 0.7
        self._send_cmd(f"SET_RPM {rpm:.1f} {-rpm:.1f}")

    def _jog_stop(self):
        self._send_cmd("SET_SPEED 0 0")

    def _emergency_stop(self):
        self._send_cmd("STOP")
        self._append_log("[EMERGENCY] STOP command sent!")

    # =========================================================================
    # BLE Connection & Command Dispatch
    # =========================================================================
    def _toggle_connection(self):
        if not self.is_connected:
            target = self.entry_device.get().strip() or "AGV_ESP32"
            self.tx_queue.put({"action": "connect", "target_name": target})
        else:
            self.tx_queue.put({"action": "disconnect"})

    def _send_cmd(self, cmd_str: str):
        self.tx_queue.put({"action": "send", "cmd": cmd_str})

    def _send_custom_command(self):
        text = self.entry_cmd.get().strip()
        if text:
            self._send_cmd(text)
            self.entry_cmd.delete(0, "end")

    def _clear_log(self):
        self.txt_console.delete("1.0", "end")

    def _append_log(self, text: str):
        self.txt_console.insert("end", text + "\n")
        self.txt_console.see("end")

    # =========================================================================
    # Polling Incoming Events from BLE Worker (Runs in GUI Main Thread)
    # =========================================================================
    def _poll_ble_events(self):
        while not self.rx_queue.empty():
            try:
                event_data = self.rx_queue.get_nowait()
                event_type = event_data.get("event")

                if event_type == "status":
                    st = event_data.get("status")
                    msg = event_data.get("msg", "")
                    self._append_log(f"[STATUS] {msg}")

                    if st == "CONNECTED":
                        self.is_connected = True
                        mtu = event_data.get("mtu", 256)
                        self.lbl_status.configure(text=f"🟢 CONNECTED (MTU: {mtu})", text_color="#00E676")
                        self.btn_connect.configure(text="Disconnect", fg_color="#C62828")
                    elif st == "DISCONNECTED":
                        self.is_connected = False
                        self.lbl_status.configure(text="🔴 DISCONNECTED", text_color="#FF5252")
                        self.btn_connect.configure(text="Scan & Connect", fg_color="#1976D2")
                    elif st in ("SCANNING", "CONNECTING"):
                        self.lbl_status.configure(text=f"🟡 {st}...", text_color="#FFD600")

                elif event_type == "tx":
                    self._append_log(f">> {event_data.get('cmd')}")

                elif event_type == "error":
                    self._append_log(f"[ERROR] {event_data.get('msg')}")

                elif event_type == "telemetry":
                    parsed = event_data.get("parsed", {})
                    p_type = parsed.get("type")
                    raw_str = event_data.get("raw", "")

                    if p_type == "LINE":
                        data: LineTelemetry = parsed["data"]
                        self._handle_line_telemetry(data)
                    elif p_type == "STEP":
                        step: StepPoint = parsed["data"]
                        self._handle_step_point(step)
                    elif p_type == "ENCODER":
                        self.lbl_enc.configure(text=f"Encoder: Left: {parsed['left']} | Right: {parsed['right']}")
                        self._append_log(raw_str)
                    elif p_type == "MISSION":
                        self.card_mission_status.configure(text=f"Mission Status: {parsed['state']} | Step: {parsed['step']}")
                        self._append_log(raw_str)
                    elif p_type in ("RESP", "ACK", "ERR", "LOG"):
                        self._append_log(raw_str)
                    else:
                        self._append_log(raw_str)

            except queue.Empty:
                break

        # Re-arm polling
        self.root.after(20, self._poll_ble_events)

    def _handle_line_telemetry(self, data: LineTelemetry):
        # Update 5 progress bars (norms are 0..1000)
        for i in range(5):
            val_norm = data.norm_values[i] / 1000.0 if i < len(data.norm_values) else 0.0
            val_raw = data.raw_values[i] if i < len(data.raw_values) else 0
            self.ir_bars[i].set(max(0.0, min(1.0, val_norm)))
            self.ir_labels[i].configure(text=f"IR{i}: Raw {val_raw:4d} | {val_norm*100:3.0f}%")

        # Update line status and error
        if data.line_seen:
            self.lbl_line_detect.configure(text="Line Status: 🟢 TRACKING LINE", text_color="#00E676")
            self.lbl_line_err.configure(text=f"Error (e): {data.line_error:+.1f} mm", text_color="#E0E0E0")
        else:
            self.lbl_line_detect.configure(text="Line Status: ⚪ NO LINE SEEN", text_color="#FFA726")
            self.lbl_line_err.configure(text="Error (e): --- mm", text_color="#757575")

        # Calibration accumulation if active
        if self.is_calibrating:
            for i in range(5):
                r = data.raw_values[i]
                if r < self.calib_min[i]: self.calib_min[i] = r
                if r > self.calib_max[i]: self.calib_max[i] = r
                delta = self.calib_max[i] - self.calib_min[i]
                self.calib_rows[i][0].configure(text=f"{self.calib_min[i]}")
                self.calib_rows[i][1].configure(text=f"{self.calib_max[i]}")
                self.calib_rows[i][2].configure(text=f"{delta}")
                if delta >= 100:
                    self.calib_rows[i][3].configure(text="PASS", text_color="#00E676")
                else:
                    self.calib_rows[i][3].configure(text="LOW", text_color="#FFA726")

    def _handle_step_point(self, step: StepPoint):
        self.step_data_x.append(step.time_ms)
        self.step_data_y.append(step.speed_rpm)
        self._update_plot(self.step_data_x, self.step_data_y)

        # Update metrics live
        if self.step_data_y:
            peak = max(self.step_data_y)
            steady = sum(self.step_data_y[-10:]) / min(len(self.step_data_y), 10)
            self.lbl_step_metrics.configure(
                text=f"Metrics:\n• Peak: {peak:.1f} RPM\n• Steady: {steady:.1f} RPM\n• Duration: {step.time_ms:.0f} ms"
            )

    def run(self):
        self.root.mainloop()


# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == "__main__":
    app = AgvDashboardApp()
    app.run()
