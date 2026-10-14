# AEV Testing Suite, Diagnostics & Ground Station GUI (`App/Testing`)

## 1. Overview
The **AEV Testing Suite** provides standalone pre-flight hardware self-tests, live serial packet inspection tools, and a dark-mode graphical ground station for bench-testing the STM32 Low-Level Controller from a laptop or NVIDIA Jetson.

---

## 2. Python Ground Station GUI (`aev_diagnostic_gui.py`)

A full-featured Python Tkinter application is included to exercise all vehicle functions and inspect live telemetry over UART (`115200 8N1`).

### 2.1 Launching the GUI
```powershell
python App\Testing\aev_diagnostic_gui.py
```

### 2.2 Dashboard Architecture (Tab 1: Control Deck & Instruments)
The main window is organized into a side-by-side cockpit:

```
┌────────────────────────────────────────┬────────────────────────────────────────┐
│  🎮 VEHICLE TRANSMISSION & COMMAND     │  📊 REAL-TIME VEHICLE TELEMETRY        │
├────────────────────────────────────────┼────────────────────────────────────────┤
│ • Transmission: Neutral / Fwd / Rev    │ • System State: INIT/CALIB/ACTIVE/FAULT│
│ • Steering Angle Slider (-30° to +30°) │ • Autonomy Serial Link: ONLINE/OFFLINE │
│ • Throttle Effort Slider (0% to 100%)  │ • Potentiometer Meter (0.0V - 3.3V)    │
│ • Braking Selector: Off / Active / Emer│ • Live Metrics: Angle, Effort, DACs   │
│ • Big Red EMERGENCY STOP (E-STOP)      │ • System Stats: Uptime, TX/RX, Errors  │
│ • Streaming Mode: Start 50 Hz / Step   │                                        │
├────────────────────────────────────────┴────────────────────────────────────────┤
│  📜 LIVE SERIAL PACKET CONSOLE (Hex Bytes + Parsed Engineering Values)          │
└─────────────────────────────────────────────────────────────────────────────────┘
```

1. **Auto-Forward Transmission**:
   - The transmission defaults to **Forward (1)** on startup.
   - Moving the Throttle Slider automatically selects Forward gear and transmits the updated packet, preventing accidental neutral lockouts during bench testing.
2. **Watchdog Heartbeat Streaming**:
   - Clicking **"Start 50 Hz Streaming"** starts a background thread transmitting control packets every 20 ms ($50\text{ Hz}$), satisfying the STM32's 200 ms watchdog.
3. **Emergency Stop (E-STOP)**:
   - Clicking the high-visibility **E-STOP** button immediately zeroes throttle, applies full emergency brake ($3.3\text{ V}$), and sends `estop = 1` to halt the vehicle.
4. **Live Packet Log**:
   - Color-coded text box displaying sent packets (`TX` in blue), received telemetry (`RX` in cyan), and system alerts (`SYS`/`ERR` in amber/red).

---

## 3. Protocol Architecture Guide (Tab 2)
The GUI embeds an interactive, scrollable protocol visualizer showing:
- **Binary Envelope**: `SOF1 (0xAA)`, `SOF2 (0x55)`, `MSG_ID`, `SEQ`, `LEN`, `PAYLOAD`, `CRC-16`, `EOF1 (0x55)`, `EOF2 (0xAA)`.
- **Memory Maps**: Byte offsets, C data types, units, and ranges for both `ControlPayload_t` ($16\text{ bytes}$) and `TelemetryPayload_t` ($26\text{ bytes}$).
- **Failsafe Rules**: Watchdog timeouts, sensor boundaries, and recovery behaviors.

---

## 4. Hardware Self-Test Engine (`system_test.c`)

For headless bench testing without the GUI, [`system_test.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing/system_test.c) runs a 4-stage automated hardware audit:

1. **Sensors Test**: Samples steering potentiometer voltage and checks against thresholds ($0.15\text{V} \le V \le 3.15\text{V}$).
2. **Actuators Test**: Tests DAC1 registers (Throttle/Brake) and TIM3 PWM compare registers, then restores safe state.
3. **Communication Test**: Verifies USART3 peripheral state and circular DMA buffer health.
4. **Motion Control Status**: Checks calibration limits and active control mode.

### Triggering via STM32CubeIDE Live Expressions
1. In STM32CubeIDE Live Expressions, add variable:
   ```c
   run_system_test
   ```
2. Set value to **`1`**.
3. Open **HTerm** (`115200 8N1`, ASCII view) to read the generated diagnostic report.

---

## 5. Repository Push Script (`push_to_github.bat`)

A 1-click batch script [`push_to_github.bat`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/push_to_github.bat) is located in the project root:
- Automatically loads portable Git into the environment session.
- Runs `git push -u origin main` with browser authentication.
