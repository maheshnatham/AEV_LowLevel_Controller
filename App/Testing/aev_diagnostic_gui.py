"""
================================================================================
AEV Low-Level Controller - Diagnostic GUI & Protocol Inspector
================================================================================
A real-time telemetry dashboard, hardware control deck, and protocol inspector
for pair-testing the STM32 NUCLEO-H723ZG Autonomous Electric Vehicle (AEV) controller
from a host laptop or NVIDIA Jetson.

Features:
  - Real-time 50 Hz bidirectional UART communication (115200 8N1).
  - Control Deck: Gear direction, steering angle slider, throttle slider, brake, E-Stop.
  - Streaming modes: Continuous (50 Hz / 20 Hz) or Single-Shot packet.
  - Telemetry Dashboard: System state, live steering pot voltage meter, angles, DACs.
  - Live Packet Log: Color-coded raw Hex + decoded engineering values.
  - Embedded Protocol Visualizer: Interactive documentation of framing and memory layout.
================================================================================
"""

import sys
import os
import time
import struct
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    messagebox.showerror(
        "Missing Dependency",
        "The 'pyserial' package is required.\nPlease install it via:\n\npip install pyserial"
    )
    sys.exit(1)

# ---------------------------------------------------------------------------
# COLOR PALETTE & STYLES (Modern Dark Theme)
# ---------------------------------------------------------------------------
BG_DARK        = "#0F172A"  # Main window background
BG_CARD        = "#1E293B"  # Card / Panel background
BG_CARD_ALT    = "#141E2F"  # Sub-card background
BORDER_COLOR   = "#334155"  # Subtle dividers
TEXT_WHITE     = "#F8FAFC"  # High contrast text
TEXT_MUTED     = "#94A3B8"  # Subtle secondary text
TEXT_DIM       = "#64748B"  # Inactive text

ACCENT_CYAN    = "#06B6D4"  # Telemetry / Status highlights
ACCENT_GREEN   = "#10B981"  # Active / Healthy / Online
ACCENT_AMBER   = "#F59E0B"  # Warning / Calibrating
ACCENT_RED     = "#EF4444"  # Fault / Emergency Stop / Errors
ACCENT_BLUE    = "#3B82F6"  # Control actions / Commands
ACCENT_PURPLE  = "#8B5CF6"  # System info

FONT_TITLE     = ("Segoe UI", 13, "bold")
FONT_HEADING   = ("Segoe UI", 10, "bold")
FONT_BODY      = ("Segoe UI", 9)
FONT_BOLD      = ("Segoe UI", 9, "bold")
FONT_NUM_LG    = ("Segoe UI", 16, "bold")
FONT_NUM_MD    = ("Segoe UI", 12, "bold")
FONT_MONO      = ("Consolas", 9)
FONT_MONO_SM   = ("Consolas", 8)

# ---------------------------------------------------------------------------
# PROTOCOL DEFINITIONS
# ---------------------------------------------------------------------------
SOF1 = 0xAA
SOF2 = 0x55
EOF1 = 0x55
EOF2 = 0xAA

MSG_ID_CMD_CONTROL = 0x01
MSG_ID_TLM_STATUS  = 0x81

CMD_PAYLOAD_LEN = 16  # sizeof(ControlPayload_t)
CMD_PACKET_LEN  = 24  # 5 header + 16 payload + 2 CRC + 2 EOF

TLM_PAYLOAD_LEN = 26  # sizeof(TelemetryPayload_t)
TLM_PACKET_LEN  = 35  # 5 header + 26 payload + 2 CRC + 2 EOF

SYSTEM_STATES = {
    0: ("INIT", TEXT_MUTED),
    1: ("CALIBRATING", ACCENT_AMBER),
    2: ("ACTIVE", ACCENT_GREEN),
    3: ("FAULT", ACCENT_RED),
}

# ---------------------------------------------------------------------------
# CRC-16-CCITT (Poly 0x1021, Init 0xFFFF)
# ---------------------------------------------------------------------------
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# SERIAL COMMUNICATION MANAGER (Threaded TX & RX)
# ---------------------------------------------------------------------------
class SerialManager:
    def __init__(self, rx_queue: queue.Queue, log_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.log_queue = log_queue
        self.ser = None
        self.running = False
        self.streaming = False
        self.stream_rate_hz = 50

        # Current setpoints to transmit
        self.drive_direction = 1      # 0=Neutral, 1=Forward, 2=Reverse
        self.steer_direction = 0      # -1=Left, 0=Center, 1=Right
        self.steer_angle = 0.0        # -30.0 to +30.0 deg
        self.acceleration = 0.0       # 0.0 to 100.0 %
        self.brake = 0                # 0=Off, 1=On, 2=Emergency
        self.estop = 0                # 0=Normal, 1=E-STOP

        self.seq = 0
        self.tx_count = 0
        self.rx_count = 0
        self.crc_err_count = 0

        self.rx_thread = None
        self.tx_thread = None

    def connect(self, port: str, baud: int = 115200) -> bool:
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_worker, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()
            self.log_queue.put(("SYS", f"Connected to {port} at {baud} baud (8N1)."))
            return True
        except Exception as e:
            self.log_queue.put(("ERR", f"Failed to connect to {port}: {e}"))
            return False

    def disconnect(self):
        self.running = False
        self.streaming = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.log_queue.put(("SYS", "Serial port disconnected."))

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def build_packet(self) -> bytes:
        """Packs ControlPayload_t into a 24-byte framed packet."""
        timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF

        payload = struct.pack(
            "<BbffBBI",
            int(self.drive_direction),
            int(self.steer_direction),
            float(self.steer_angle),
            float(self.acceleration),
            int(self.brake),
            int(self.estop),
            int(timestamp_ms)
        )

        msg_id = MSG_ID_CMD_CONTROL
        payload_len = len(payload) # 16

        # CRC calculated over [MSG_ID, SEQ, LEN, ...PAYLOAD...]
        crc_data = bytes([msg_id, self.seq, payload_len]) + payload
        crc = calculate_crc16(crc_data)

        pkt = bytearray()
        pkt.append(SOF1)
        pkt.append(SOF2)
        pkt.append(msg_id)
        pkt.append(self.seq)
        pkt.append(payload_len)
        pkt.extend(payload)
        pkt.append(crc & 0xFF)
        pkt.append((crc >> 8) & 0xFF)
        pkt.append(EOF1)
        pkt.append(EOF2)

        self.seq = (self.seq + 1) & 0xFF
        return bytes(pkt)

    def send_single_packet(self):
        """Sends a single packet immediately (for step-testing)."""
        if not self.is_connected():
            return
        try:
            pkt = self.build_packet()
            self.ser.write(pkt)
            self.tx_count += 1
            hex_str = " ".join(f"{b:02X}" for b in pkt)
            info = f"Dir={self.drive_direction} Steer={self.steer_angle:+.1f}° Throt={self.acceleration:.0f}% Brk={self.brake} EStop={self.estop}"
            self.log_queue.put(("TX", f"[TX #{self.tx_count}] {hex_str} | {info}"))
        except Exception as e:
            self.log_queue.put(("ERR", f"TX Error: {e}"))

    def _tx_worker(self):
        """Streaming background thread."""
        while self.running:
            if self.streaming and self.is_connected():
                try:
                    pkt = self.build_packet()
                    self.ser.write(pkt)
                    self.tx_count += 1

                    # Log periodically (every ~10 packets to prevent flooding the GUI)
                    if self.tx_count % 10 == 0:
                        hex_str = " ".join(f"{b:02X}" for b in pkt)
                        info = f"Dir={self.drive_direction} Steer={self.steer_angle:+.1f}° Throt={self.acceleration:.0f}%"
                        self.log_queue.put(("TX", f"[TX #{self.tx_count}] {hex_str} | {info}"))

                    interval = 1.0 / max(1, self.stream_rate_hz)
                    time.sleep(interval)
                except Exception as e:
                    self.log_queue.put(("ERR", f"Streaming error: {e}"))
                    time.sleep(0.1)
            else:
                time.sleep(0.02)

    def _rx_worker(self):
        """Background thread: reads raw bytes and parses 35-byte telemetry frames."""
        rx_buf = bytearray()
        while self.running and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                rx_buf.extend(chunk)

                # Process buffer searching for 35-byte telemetry frames
                while len(rx_buf) >= TLM_PACKET_LEN:
                    # Look for Start-Of-Frame
                    if rx_buf[0] == SOF1 and rx_buf[1] == SOF2:
                        msg_id = rx_buf[2]
                        seq = rx_buf[3]
                        plen = rx_buf[4]

                        if msg_id == MSG_ID_TLM_STATUS and plen == TLM_PAYLOAD_LEN:
                            frame = rx_buf[:TLM_PACKET_LEN]

                            # Verify EOF
                            if frame[33] == EOF1 and frame[34] == EOF2:
                                # Verify CRC-16
                                calc_crc = calculate_crc16(frame[2:31])
                                rx_crc = (frame[32] << 8) | frame[31]

                                if calc_crc == rx_crc:
                                    payload = bytes(frame[5:31])
                                    state, steer_ang, pot_v, effort, throt_v, brake_v, alive, uptime = struct.unpack("<BfffffBI", payload)

                                    self.rx_count += 1
                                    tlm_dict = {
                                        "state": state,
                                        "steer_angle": steer_ang,
                                        "pot_v": pot_v,
                                        "effort": effort,
                                        "throt_v": throt_v,
                                        "brake_v": brake_v,
                                        "alive": alive,
                                        "uptime": uptime,
                                        "seq": seq,
                                        "raw_hex": " ".join(f"{b:02X}" for b in frame)
                                    }
                                    self.rx_queue.put(tlm_dict)

                                    # Log packet (every 10 frames to avoid bogging down GUI)
                                    if self.rx_count % 10 == 0:
                                        summary = f"State={state} Pot={pot_v:.3f}V SteerEffort={effort:.2f} Brk={brake_v:.2f}V Link={alive}"
                                        self.log_queue.put(("RX", f"[RX #{self.rx_count}] {tlm_dict['raw_hex'][:40]}... | {summary}"))
                                else:
                                    self.crc_err_count += 1
                                    self.log_queue.put(("ERR", f"[RX CRC ERR] Calc=0x{calc_crc:04X} vs Rx=0x{rx_crc:04X}"))

                                del rx_buf[:TLM_PACKET_LEN]
                                continue
                            else:
                                # Malformed EOF, slide by 1 byte
                                del rx_buf[0]
                        else:
                            # Unexpected msg_id or length, slide by 1 byte
                            del rx_buf[0]
                    else:
                        del rx_buf[0]

            except Exception as e:
                if self.running:
                    self.log_queue.put(("ERR", f"RX Exception: {e}"))
                time.sleep(0.05)


# ---------------------------------------------------------------------------
# MAIN APPLICATION GUI
# ---------------------------------------------------------------------------
class AEVDiagnosticApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AEV Low-Level Controller - Hardware Testing Deck & Protocol Inspector")
        self.root.geometry("1180x820")
        self.root.minsize(1050, 750)
        self.root.configure(bg=BG_DARK)

        self.rx_queue = queue.Queue()
        self.log_queue = queue.Queue()
        self.manager = SerialManager(self.rx_queue, self.log_queue)

        self.last_tlm_time = 0
        self.prev_rx_count = 0
        self.rx_rate_hz = 0

        self._init_styles()
        self._build_header()
        self._build_tabs()
        self._build_statusbar()

        # Start periodic GUI refresh loops
        self.root.after(50, self._process_rx_queue)
        self.root.after(100, self._process_log_queue)
        self.root.after(1000, self._calc_packet_rates)

    def _init_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        # Global tab styling
        style.configure("TNotebook", background=BG_DARK, borderwidth=0)
        style.configure("TNotebook.Tab", background=BG_CARD, foreground=TEXT_MUTED,
                        padding=[16, 8], font=FONT_BOLD, borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", ACCENT_CYAN)],
                  foreground=[("selected", BG_DARK)])

        style.configure("TProgressbar", thickness=18, troughcolor=BG_CARD_ALT, background=ACCENT_CYAN)

    def _build_header(self):
        header_frame = tk.Frame(self.root, bg=BG_CARD, padx=16, pady=10, relief="flat", highlightbackground=BORDER_COLOR, highlightthickness=1)
        header_frame.pack(fill="x", padx=12, pady=(10, 6))

        # Title and Badge
        title_box = tk.Frame(header_frame, bg=BG_CARD)
        title_box.pack(side="left")

        title_lbl = tk.Label(title_box, text="⚡ AEV LOW-LEVEL CONTROLLER", font=FONT_TITLE, fg=TEXT_WHITE, bg=BG_CARD)
        title_lbl.pack(side="left")

        sub_lbl = tk.Label(title_box, text="STM32H723ZG TEST BENCH", font=FONT_MONO_SM, fg=ACCENT_CYAN, bg=BG_CARD_ALT, padx=6, pady=2)
        sub_lbl.pack(side="left", padx=10)

        # Port Selection & Connect Box
        conn_box = tk.Frame(header_frame, bg=BG_CARD)
        conn_box.pack(side="right")

        tk.Label(conn_box, text="Port:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(side="left", padx=(0, 4))
        self.port_combo = ttk.Combobox(conn_box, width=12, state="readonly", font=FONT_BODY)
        self.port_combo.pack(side="left", padx=4)
        self._refresh_ports()

        btn_refresh = tk.Button(conn_box, text="🔄", command=self._refresh_ports, font=FONT_BODY,
                                bg=BG_CARD_ALT, fg=TEXT_WHITE, activebackground=BORDER_COLOR, relief="flat", bd=0, padx=6)
        btn_refresh.pack(side="left", padx=2)

        tk.Label(conn_box, text="Baud:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(side="left", padx=(8, 4))
        self.baud_combo = ttk.Combobox(conn_box, width=8, state="readonly", font=FONT_BODY,
                                       values=["115200", "57600", "921600"])
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left", padx=4)

        self.btn_connect = tk.Button(conn_box, text="Connect", command=self._toggle_connection,
                                     font=FONT_BOLD, bg=ACCENT_CYAN, fg=BG_DARK, activebackground=TEXT_WHITE,
                                     relief="flat", bd=0, padx=14, pady=4, cursor="hand2")
        self.btn_connect.pack(side="left", padx=10)

        # Connection status pill
        self.lbl_conn_status = tk.Label(conn_box, text="DISCONNECTED", font=FONT_MONO_SM, fg=TEXT_DIM,
                                        bg=BG_CARD_ALT, padx=8, pady=4, relief="flat")
        self.lbl_conn_status.pack(side="left")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            # Auto-select STLink VCP if present
            stlink_ports = [p.device for p in serial.tools.list_ports.comports() if "STLink" in p.description or "Virtual" in p.description]
            if stlink_ports:
                self.port_combo.set(stlink_ports[0])
            else:
                self.port_combo.set(ports[0])
        else:
            self.port_combo.set("No Ports")

    def _toggle_connection(self):
        if not self.manager.is_connected():
            port = self.port_combo.get()
            if not port or port == "No Ports":
                messagebox.showwarning("Warning", "Please select a valid COM port!")
                return
            baud = int(self.baud_combo.get())
            if self.manager.connect(port, baud):
                self.btn_connect.config(text="Disconnect", bg=ACCENT_RED, fg=TEXT_WHITE)
                self.lbl_conn_status.config(text=f"CONNECTED ({port})", fg=ACCENT_GREEN)
        else:
            self.manager.disconnect()
            self.btn_connect.config(text="Connect", bg=ACCENT_CYAN, fg=BG_DARK)
            self.lbl_conn_status.config(text="DISCONNECTED", fg=TEXT_DIM)
            self.lbl_link_alive.config(text="OFFLINE", fg=TEXT_DIM)

    def _build_tabs(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=12, pady=4)

        # Tab 1: Live Testing Deck & Telemetry
        self.tab_dashboard = tk.Frame(self.notebook, bg=BG_DARK)
        self.notebook.add(self.tab_dashboard, text="  🎛️  Control Deck & Instruments  ")

        # Tab 2: Protocol Architecture Visualizer
        self.tab_protocol = tk.Frame(self.notebook, bg=BG_DARK)
        self.notebook.add(self.tab_protocol, text="  📖  Protocol Architecture Guide  ")

        self._build_dashboard_tab()
        self._build_protocol_tab()

    # -----------------------------------------------------------------------
    # TAB 1: DASHBOARD & CONTROL DECK
    # -----------------------------------------------------------------------
    def _build_dashboard_tab(self):
        paned = tk.PanedWindow(self.tab_dashboard, orient="vertical", bg=BG_DARK, sashwidth=4, bd=0)
        paned.pack(fill="both", expand=True, padx=4, pady=4)

        # Upper section: Controls & Instruments side-by-side
        top_split = tk.Frame(paned, bg=BG_DARK)
        paned.add(top_split, height=440)

        # Left Column: Command & Control Deck
        self._build_control_deck(top_split)

        # Right Column: Live Telemetry Instruments
        self._build_instrument_panel(top_split)

        # Lower section: Live Communication Console
        bottom_frame = tk.Frame(paned, bg=BG_CARD, padx=8, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        paned.add(bottom_frame, height=240)
        self._build_log_console(bottom_frame)

    def _build_control_deck(self, parent):
        deck = tk.LabelFrame(parent, text="  🎮  VEHICLE TRANSMISSION & COMMAND DECK  ", font=FONT_HEADING,
                             fg=ACCENT_CYAN, bg=BG_CARD, padx=14, pady=10, relief="flat", highlightbackground=BORDER_COLOR, highlightthickness=1)
        deck.pack(side="left", fill="both", expand=True, padx=(0, 6))

        # 1. Drive Direction (Gear)
        dir_frame = tk.Frame(deck, bg=BG_CARD)
        dir_frame.pack(fill="x", pady=(0, 8))
        tk.Label(dir_frame, text="Transmission:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(side="left")

        self.gear_var = tk.IntVar(value=1)
        gears = [("Neutral (0)", 0), ("Forward (1)", 1), ("Reverse (2)", 2)]
        for text, val in gears:
            rb = tk.Radiobutton(dir_frame, text=text, variable=self.gear_var, value=val,
                                command=self._on_control_change, font=FONT_BODY, fg=TEXT_WHITE, bg=BG_CARD,
                                activebackground=BG_CARD, activeforeground=ACCENT_CYAN, selectcolor=BG_CARD_ALT)
            rb.pack(side="left", padx=10)

        # 2. Steering Angle Slider (-30.0° to +30.0°)
        steer_box = tk.Frame(deck, bg=BG_CARD)
        steer_box.pack(fill="x", pady=6)

        steer_hdr = tk.Frame(steer_box, bg=BG_CARD)
        steer_hdr.pack(fill="x")
        tk.Label(steer_hdr, text="Target Steering Angle:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(side="left")
        self.lbl_steer_val = tk.Label(steer_hdr, text="0.0°", font=FONT_NUM_MD, fg=ACCENT_CYAN, bg=BG_CARD)
        self.lbl_steer_val.pack(side="right")

        self.slider_steer = tk.Scale(steer_box, from_=-30.0, to=30.0, resolution=0.5, orient="horizontal",
                                     command=self._on_steer_slide, bg=BG_CARD_ALT, fg=TEXT_WHITE,
                                     troughcolor=BG_DARK, highlightthickness=0, activebackground=ACCENT_CYAN, showvalue=0)
        self.slider_steer.set(0.0)
        self.slider_steer.pack(fill="x", pady=4)

        btn_center = tk.Button(steer_box, text="Center Wheel (0.0°)", command=lambda: self.slider_steer.set(0.0),
                               font=FONT_MONO_SM, bg=BG_CARD_ALT, fg=TEXT_MUTED, relief="flat", padx=8, pady=2)
        btn_center.pack(anchor="e")

        # 3. Throttle Slider (0 to 100%)
        throt_box = tk.Frame(deck, bg=BG_CARD)
        throt_box.pack(fill="x", pady=6)

        throt_hdr = tk.Frame(throt_box, bg=BG_CARD)
        throt_hdr.pack(fill="x")
        tk.Label(throt_hdr, text="Throttle Effort:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(side="left")
        self.lbl_throt_val = tk.Label(throt_hdr, text="0.0 %", font=FONT_NUM_MD, fg=ACCENT_GREEN, bg=BG_CARD)
        self.lbl_throt_val.pack(side="right")

        self.slider_throt = tk.Scale(throt_box, from_=0.0, to=100.0, resolution=1.0, orient="horizontal",
                                     command=self._on_throt_slide, bg=BG_CARD_ALT, fg=TEXT_WHITE,
                                     troughcolor=BG_DARK, highlightthickness=0, activebackground=ACCENT_GREEN, showvalue=0)
        self.slider_throt.set(0.0)
        self.slider_throt.pack(fill="x", pady=4)

        # 4. Brake Selector & E-Stop
        action_row = tk.Frame(deck, bg=BG_CARD)
        action_row.pack(fill="x", pady=8)

        brake_box = tk.Frame(action_row, bg=BG_CARD)
        brake_box.pack(side="left", fill="x", expand=True)
        tk.Label(brake_box, text="Braking Mode:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD).pack(anchor="w")

        self.brake_var = tk.IntVar(value=0)
        b_modes = [("Off (0)", 0), ("Active (1)", 1), ("Emergency (2)", 2)]
        for text, val in b_modes:
            rb = tk.Radiobutton(brake_box, text=text, variable=self.brake_var, value=val,
                                command=self._on_control_change, font=FONT_BODY, fg=TEXT_WHITE, bg=BG_CARD,
                                activebackground=BG_CARD, selectcolor=BG_CARD_ALT)
            rb.pack(anchor="w", padx=4)

        # Big Panic Emergency Stop Button
        self.btn_estop = tk.Button(action_row, text="🚨 EMERGENCY\nSTOP (E-STOP)", command=self._trigger_estop,
                                   font=FONT_HEADING, bg=ACCENT_RED, fg=TEXT_WHITE, activebackground="#B91C1C",
                                   relief="flat", bd=0, padx=16, pady=8, cursor="hand2")
        self.btn_estop.pack(side="right", padx=10)

        # 5. Transmission Loop Mode
        mode_box = tk.Frame(deck, bg=BG_CARD_ALT, padx=10, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        mode_box.pack(fill="x", pady=(10, 0))

        tk.Label(mode_box, text="Stream Mode:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD_ALT).pack(side="left")

        self.btn_stream = tk.Button(mode_box, text="Start 50 Hz Streaming", command=self._toggle_stream,
                                    font=FONT_BOLD, bg=BG_CARD, fg=ACCENT_CYAN, activebackground=ACCENT_CYAN,
                                    relief="flat", bd=0, padx=10, pady=4, cursor="hand2")
        self.btn_stream.pack(side="left", padx=8)

        btn_step = tk.Button(mode_box, text="Send 1 Packet", command=self._on_send_single,
                             font=FONT_BODY, bg=BG_CARD, fg=TEXT_WHITE, activebackground=BORDER_COLOR,
                             relief="flat", bd=0, padx=10, pady=4, cursor="hand2")
        btn_step.pack(side="right")

    def _build_instrument_panel(self, parent):
        panel = tk.LabelFrame(parent, text="  📊  REAL-TIME VEHICLE TELEMETRY  ", font=FONT_HEADING,
                              fg=ACCENT_GREEN, bg=BG_CARD, padx=14, pady=10, relief="flat", highlightbackground=BORDER_COLOR, highlightthickness=1)
        panel.pack(side="right", fill="both", expand=True, padx=(6, 0))

        # Top row: State Pill & Link Health
        status_row = tk.Frame(panel, bg=BG_CARD)
        status_row.pack(fill="x", pady=(0, 10))

        # System State Card
        card_state = tk.Frame(status_row, bg=BG_CARD_ALT, padx=12, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        card_state.pack(side="left", fill="both", expand=True, padx=(0, 4))
        tk.Label(card_state, text="SYSTEM STATE", font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_CARD_ALT).pack(anchor="w")
        self.lbl_sys_state = tk.Label(card_state, text="AWAITING DATA", font=FONT_NUM_MD, fg=TEXT_DIM, bg=BG_CARD_ALT)
        self.lbl_sys_state.pack(anchor="w", pady=(2, 0))

        # Watchdog Link Health Card
        card_link = tk.Frame(status_row, bg=BG_CARD_ALT, padx=12, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        card_link.pack(side="right", fill="both", expand=True, padx=(4, 0))
        tk.Label(card_link, text="AUTONOMY SERIAL LINK", font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_CARD_ALT).pack(anchor="w")
        self.lbl_link_alive = tk.Label(card_link, text="OFFLINE", font=FONT_NUM_MD, fg=TEXT_DIM, bg=BG_CARD_ALT)
        self.lbl_link_alive.pack(anchor="w", pady=(2, 0))

        # Potentiometer Sensor Gauge
        pot_card = tk.Frame(panel, bg=BG_CARD_ALT, padx=12, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        pot_card.pack(fill="x", pady=4)

        pot_hdr = tk.Frame(pot_card, bg=BG_CARD_ALT)
        pot_hdr.pack(fill="x")
        tk.Label(pot_hdr, text="Steering Potentiometer Voltage:", font=FONT_BOLD, fg=TEXT_MUTED, bg=BG_CARD_ALT).pack(side="left")
        self.lbl_pot_v = tk.Label(pot_hdr, text="0.000 V", font=FONT_NUM_MD, fg=ACCENT_CYAN, bg=BG_CARD_ALT)
        self.lbl_pot_v.pack(side="right")

        self.pot_progress = ttk.Progressbar(pot_card, orient="horizontal", mode="determinate", maximum=3.3)
        self.pot_progress.pack(fill="x", pady=6)

        pot_legend = tk.Frame(pot_card, bg=BG_CARD_ALT)
        pot_legend.pack(fill="x")
        tk.Label(pot_legend, text="0.0V (Left Hardstop)", font=FONT_MONO_SM, fg=TEXT_DIM, bg=BG_CARD_ALT).pack(side="left")
        tk.Label(pot_legend, text="1.65V (Center)", font=FONT_MONO_SM, fg=TEXT_DIM, bg=BG_CARD_ALT).pack(side="left", expand=True)
        tk.Label(pot_legend, text="3.3V (Right Hardstop)", font=FONT_MONO_SM, fg=TEXT_DIM, bg=BG_CARD_ALT).pack(side="right")

        # Actuator Feedback Grid
        grid_frame = tk.Frame(panel, bg=BG_CARD)
        grid_frame.pack(fill="x", pady=8)

        def make_metric(col, title, default_val, fg_col):
            frame = tk.Frame(grid_frame, bg=BG_CARD_ALT, padx=10, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
            frame.grid(row=0, column=col, sticky="nsew", padx=3)
            grid_frame.columnconfigure(col, weight=1)
            tk.Label(frame, text=title, font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_CARD_ALT).pack(anchor="w")
            lbl = tk.Label(frame, text=default_val, font=FONT_NUM_MD, fg=fg_col, bg=BG_CARD_ALT)
            lbl.pack(anchor="w", pady=(2, 0))
            return lbl

        self.lbl_est_angle   = make_metric(0, "EST. ANGLE", "0.00°", ACCENT_CYAN)
        self.lbl_steer_eff   = make_metric(1, "MOTOR EFFORT", "0.00", ACCENT_PURPLE)
        self.lbl_throt_dac   = make_metric(2, "THROTTLE DAC", "0.00 V", ACCENT_GREEN)
        self.lbl_brake_dac   = make_metric(3, "BRAKE DAC", "0.00 V", ACCENT_RED)

        # Bottom System Stats Box
        stats_frame = tk.Frame(panel, bg=BG_CARD_ALT, padx=10, pady=8, highlightbackground=BORDER_COLOR, highlightthickness=1)
        stats_frame.pack(fill="x", pady=(8, 0))

        self.lbl_stats = tk.Label(stats_frame, text="Uptime: 0s  |  TX: 0  |  RX: 0  |  Errors: 0  |  Rate: 0 Hz",
                                  font=FONT_MONO, fg=TEXT_MUTED, bg=BG_CARD_ALT)
        self.lbl_stats.pack(anchor="w")

    def _build_log_console(self, parent):
        header = tk.Frame(parent, bg=BG_CARD)
        header.pack(fill="x", pady=(0, 4))

        tk.Label(header, text="  📜  LIVE SERIAL PACKET CONSOLE (HEX & PARSED)", font=FONT_BOLD,
                 fg=TEXT_WHITE, bg=BG_CARD).pack(side="left")

        btn_clear = tk.Button(header, text="Clear", command=self._clear_log, font=FONT_MONO_SM,
                              bg=BG_CARD_ALT, fg=TEXT_MUTED, relief="flat", padx=8, pady=2)
        btn_clear.pack(side="right", padx=4)

        self.autoscroll_var = tk.BooleanVar(value=True)
        chk_scroll = tk.Checkbutton(header, text="Auto-Scroll", variable=self.autoscroll_var,
                                    font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_CARD, selectcolor=BG_CARD_ALT,
                                    activebackground=BG_CARD)
        chk_scroll.pack(side="right", padx=8)

        # Log Text Box
        log_box_frame = tk.Frame(parent, bg=BG_DARK)
        log_box_frame.pack(fill="both", expand=True)

        scrollbar = tk.Scrollbar(log_box_frame)
        scrollbar.pack(side="right", fill="y")

        self.log_text = tk.Text(log_box_frame, height=9, bg=BG_DARK, fg=TEXT_WHITE, font=FONT_MONO,
                                yscrollcommand=scrollbar.set, relief="flat", wrap="none")
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)

        # Configure color tags
        self.log_text.tag_config("TX",  foreground=ACCENT_BLUE)
        self.log_text.tag_config("RX",  foreground=ACCENT_CYAN)
        self.log_text.tag_config("ERR", foreground=ACCENT_RED)
        self.log_text.tag_config("SYS", foreground=ACCENT_AMBER)

    # -----------------------------------------------------------------------
    # TAB 2: PROTOCOL ARCHITECTURE GUIDE
    # -----------------------------------------------------------------------
    def _build_protocol_tab(self):
        canvas = tk.Canvas(self.tab_protocol, bg=BG_DARK, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.tab_protocol, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=BG_DARK, padx=20, pady=16)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        def add_card(title, text_content):
            card = tk.LabelFrame(scrollable_frame, text=f"  {title}  ", font=FONT_HEADING,
                                 fg=ACCENT_CYAN, bg=BG_CARD, padx=14, pady=12, relief="flat", highlightbackground=BORDER_COLOR, highlightthickness=1)
            card.pack(fill="x", pady=8)
            lbl = tk.Label(card, text=text_content, font=FONT_MONO, fg=TEXT_WHITE, bg=BG_CARD, justify="left", wraplength=1050)
            lbl.pack(anchor="w")

        add_card("1. Frame Architecture Overview",
"""Framing Structure (All packets wrapped in Start-of-Frame, Length, CRC, and End-of-Frame):
+--------+--------+--------+--------+--------+------------------------+--------+--------+--------+--------+
|  SOF1  |  SOF2  | MSG_ID |  SEQ   |  LEN   |   PAYLOAD (Variable)   | CRC_L  | CRC_H  |  EOF1  |  EOF2  |
|  0xAA  |  0x55  |  0x01  | 0..255 | 0x10   | [16-Byte Control Data] | CCITT-16 (Poly 0x1021)|  0x55  |  0xAA  |
+--------+--------+--------+--------+--------+------------------------+--------+--------+--------+--------+
Note: The CRC-16 is computed over [MSG_ID, SEQ, LEN, ...PAYLOAD...] (starts at Byte 2, length = 3 + LEN).""")

        add_card("2. Jetson -> STM32 Control Packet (MSG_ID = 0x01, 24 Bytes Total)",
"""Struct: ControlPayload_t (16 Bytes, packed, Little-Endian):
Offset  Type     Field Name        Values / Physical Meaning
------------------------------------------------------------------------------------------
  0     uint8    drive_direction   0 = NEUTRAL, 1 = FORWARD, 2 = REVERSE
  1     int8     steer_direction  -1 = LEFT, 0 = CENTER, +1 = RIGHT
  2..5  float    steer_angle      Target angle in degrees (-30.0 to +30.0 deg)
  6..9  float    acceleration     Target throttle percentage (0.0 to 100.0 %)
 10     uint8    brake            0 = OFF, 1 = APPLIED (1.65V), 2 = EMERGENCY (3.3V)
 11     uint8    estop            0 = NORMAL, 1 = EMERGENCY STOP ASSERTED
 12..15 uint32   timestamp_ms     Millisecond timestamp from host clock""")

        add_card("3. STM32 -> Jetson Telemetry Packet (MSG_ID = 0x81, 35 Bytes Total)",
"""Struct: TelemetryPayload_t (26 Bytes, packed, Little-Endian):
Offset  Type     Field Name             Values / Physical Meaning
------------------------------------------------------------------------------------------
  0     uint8    system_state           0=INIT, 1=CALIBRATING, 2=ACTIVE, 3=FAULT
  1..4  float    actual_steer_angle     Filtered steering angle in degrees (from EKF)
  5..8  float    steering_pot_v         Raw potentiometer ADC reading (0.0 to 3.3V)
  9..12 float    applied_steer_effort   Current steering H-bridge PWM (-1.0 to +1.0)
 13..16 float    applied_throttle_v     Current throttle DAC output (0.0 to 3.3V)
 17..20 float    applied_brake_v        Current brake DAC output (0.0 to 3.3V)
 21     uint8    link_alive             1 = Link active within 200ms, 0 = Watchdog timeout
 22..25 uint32   uptime_ms              STM32 system uptime in milliseconds (HAL_GetTick)""")

        add_card("4. Autonomous Safety & Watchdog Specification",
"""Autonomous Failsafe Watchdog:
  1. The STM32 expects control packets at 20 Hz to 50 Hz.
  2. If no valid packet arrives within 200 ms (COMM_TIMEOUT_MS):
     - STM32 sets link_ok = false and Yellow LED (COMM_LINK) turns OFF.
     - Acceleration DAC drops immediately to 0.0V (Neutral cut).
     - Mechanical brakes are automatically applied for safe deceleration.
     - Steering holds center position (0.0°).
  3. Red LED (SYS_FAULT) flashes diagnostic pulse codes (1 blink = Comm Watchdog Timeout).
  4. Transmission resumes automatically on the next valid packet.""")

    # -----------------------------------------------------------------------
    # EVENT HANDLERS & PERIODIC REFRESH
    # -----------------------------------------------------------------------
    def _on_control_change(self):
        self.manager.drive_direction = self.gear_var.get()
        self.manager.brake = self.brake_var.get()
        # Update steering direction sign
        steer = self.slider_steer.get()
        self.manager.steer_direction = 1 if steer > 0.5 else (-1 if steer < -0.5 else 0)

    def _on_steer_slide(self, val):
        angle = float(val)
        self.lbl_steer_val.config(text=f"{angle:+.1f}°")
        self.manager.steer_angle = angle
        self._on_control_change()

    def _on_throt_slide(self, val):
        pct = float(val)
        self.lbl_throt_val.config(text=f"{pct:.0f} %")
        self.manager.acceleration = pct
        # Auto-engage Forward gear if in Neutral so throttle works immediately
        if pct > 0.0 and self.gear_var.get() == 0:
            self.gear_var.set(1)
            self.manager.drive_direction = 1
        self._on_control_change()

    def _trigger_estop(self):
        """Immediately zeroes throttle, applies emergency brake, and flags E-Stop."""
        self.gear_var.set(0)
        self.slider_throt.set(0.0)
        self.brake_var.set(2)  # Emergency brake
        self.manager.estop = 1
        self.manager.drive_direction = 0
        self.manager.acceleration = 0.0
        self.manager.brake = 2
        self.manager.send_single_packet()
        self.log_queue.put(("ERR", "🚨 [PANIC] EMERGENCY STOP TRANSMITTED!"))

    def _toggle_stream(self):
        if not self.manager.is_connected():
            messagebox.showwarning("Not Connected", "Please connect to the STM32 serial port first!")
            return

        self.manager.streaming = not self.manager.streaming
        if self.manager.streaming:
            self.manager.estop = 0  # Clear estop on clean stream start
            self.btn_stream.config(text="⏹ Stop 50 Hz Stream", bg=ACCENT_AMBER, fg=BG_DARK)
            self.log_queue.put(("SYS", "Started continuous 50 Hz streaming loop."))
        else:
            self.btn_stream.config(text="Start 50 Hz Streaming", bg=BG_CARD, fg=ACCENT_CYAN)
            self.log_queue.put(("SYS", "Stopped continuous streaming."))

    def _on_send_single(self):
        if not self.manager.is_connected():
            messagebox.showwarning("Not Connected", "Please connect to the STM32 serial port first!")
            return
        self.manager.send_single_packet()

    def _process_rx_queue(self):
        """Polls telemetry queue and updates dashboard instruments."""
        while not self.rx_queue.empty():
            tlm = self.rx_queue.get_nowait()
            self.last_tlm_time = time.time()

            # 1. System State
            state_val = tlm["state"]
            name, color = SYSTEM_STATES.get(state_val, ("UNKNOWN", TEXT_MUTED))
            self.lbl_sys_state.config(text=f"{name} ({state_val})", fg=color)

            # 2. Link Watchdog
            if tlm["alive"] == 1:
                self.lbl_link_alive.config(text="ONLINE (OK)", fg=ACCENT_GREEN)
            else:
                self.lbl_link_alive.config(text="TIMEOUT (200ms)", fg=ACCENT_RED)

            # 3. Potentiometer
            pot_v = tlm["pot_v"]
            self.lbl_pot_v.config(text=f"{pot_v:.3f} V")
            self.pot_progress["value"] = min(3.3, max(0.0, pot_v))

            # 4. Actuators
            self.lbl_est_angle.config(text=f"{tlm['steer_angle']:+.2f}°")
            self.lbl_steer_eff.config(text=f"{tlm['effort']:+.2f}")
            self.lbl_throt_dac.config(text=f"{tlm['throt_v']:.2f} V")
            self.lbl_brake_dac.config(text=f"{tlm['brake_v']:.2f} V")

            # 5. Uptime & Counters
            uptime_s = tlm["uptime"] / 1000.0
            mins = int(uptime_s // 60)
            secs = uptime_s % 60
            self.lbl_stats.config(
                text=f"Uptime: {mins:02d}:{secs:04.1f}  |  TX: {self.manager.tx_count}  |  RX: {self.manager.rx_count}  |  Errors: {self.manager.crc_err_count}  |  Rate: {self.rx_rate_hz} Hz"
            )

        # Timeout indicator: if no telemetry in 0.5s while connected
        if self.manager.is_connected() and (time.time() - self.last_tlm_time) > 0.5:
            self.lbl_link_alive.config(text="SILENT (>500ms)", fg=TEXT_MUTED)

        self.root.after(40, self._process_rx_queue)

    def _process_log_queue(self):
        """Polls log messages and appends to the scrollable console."""
        while not self.log_queue.empty():
            tag, msg = self.log_queue.get_nowait()
            ts = time.strftime("%H:%M:%S")
            self.log_text.insert("end", f"[{ts}] {msg}\n", tag)

            if self.autoscroll_var.get():
                self.log_text.see("end")

        self.root.after(100, self._process_log_queue)

    def _calc_packet_rates(self):
        """Calculates actual received packet rate (Hz)."""
        delta = self.manager.rx_count - self.prev_rx_count
        self.rx_rate_hz = delta
        self.prev_rx_count = self.manager.rx_count
        self.root.after(1000, self._calc_packet_rates)

    def _clear_log(self):
        self.log_text.delete("1.0", "end")

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=BG_CARD_ALT, padx=12, pady=4)
        bar.pack(fill="x", side="bottom")

        tk.Label(bar, text="Autonomous Electric Vehicle (AEV) • Diagnostic Terminal v1.0",
                 font=FONT_MONO_SM, fg=TEXT_DIM, bg=BG_CARD_ALT).pack(side="left")
        tk.Label(bar, text="USART3 @ 115200 8N1 • Circular DMA • CRC-16 CCITT",
                 font=FONT_MONO_SM, fg=TEXT_DIM, bg=BG_CARD_ALT).pack(side="right")


# ---------------------------------------------------------------------------
# APPLICATION ENTRY POINT
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = AEVDiagnosticApp(root)
    try:
        root.mainloop()
    finally:
        if app.manager.is_connected():
            app.manager.disconnect()
