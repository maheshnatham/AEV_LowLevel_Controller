# Testing & Diagnostic Suite (`App/Testing`)

## 1. Overview
The **Testing & Diagnostic Suite** provides standalone validation routines to verify hardware connectivity, sensor health, actuator readiness, and communication throughput **before** running the vehicle.

It can be triggered:
- From the **STM32CubeIDE debugger** (setting a Live Watch variable).
- In software on startup or before arming.
- Via a dedicated hardware or serial trigger command.

---

## 2. Available Functions in `system_test.h`

| Function | Description |
|:---|:---|
| `SystemTest_RunAll()` | Executes the full 4-stage hardware self-test, prints an ASCII report to UART, and transmits 1x sample binary telemetry packet. |
| `SystemTest_CheckSensors()` | Samples the steering potentiometer via DMA and verifies voltage is within electrical thresholds ($0.15\text{ V} \le V \le 3.15\text{ V}$). |
| `SystemTest_CheckActuators()` | Tests DAC1 CH1 (Throttle), DAC1 CH2 (Brake), and TIM3 PWM (Steering), then asserts safe state. |
| `SystemTest_CheckComm()` | Verifies UART peripheral state and DMA circular buffer status. |
| `SystemTest_PrintDiagnostics()` | Emits a single-line ASCII diagnostic string for continuous terminal monitoring. |
| `SystemTest_SendSampleTlm()` | Sends a known binary telemetry frame to test serial decoding on the host computer. |

---

## 3. How to Run Initial Checks with Laptop & HTerm

### 3.1 Live Watch Trigger via STM32CubeIDE
1. Start your debugging session in STM32CubeIDE.
2. In the **Live Expressions** tab, add the variable:
   ```
   run_system_test
   ```
3. Set its value to **`1`**.
4. The main loop in [`main.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/Core/Src/main.c) detects this, clears the flag back to `0`, and immediately executes `SystemTest_RunAll()`.

### 3.2 Expected Output in HTerm (ASCII Mode)
Connect HTerm at `115200 8N1` and select **ASCII** mode in the received pane. You will see:

```text
=======================================================
    AEV LOW-LEVEL CONTROLLER SYSTEM INITIAL CHECKS    
=======================================================
[1/4] Sensors Test: Steering Pot = 1.648 V [PASS]
      -> Raw Block Average = 32742.0 counts (16-bit)
[2/4] Actuators Test: DAC1 (Throttle/Brake) & TIM3 PWM [PASS]
      -> Actuators_ApplySafeState() executed successfully.
[3/4] Comm Test: USART3 @ 115200 8N1 (Circular DMA) [PASS]
      -> Watchdog Link Status: WAITING FOR COMMAND PACKET
[4/4] Motion Control: Current State = ACTIVE (PID Closed-Loop)
      -> Calibrated Center = 1.650 V
      -> Calibrated Range  = 0.450 V to 2.850 V
      -> Scale Factor      = 29.17 deg/V
-------------------------------------------------------
Sending 1x Sample Binary Telemetry Packet (MsgID 0x81)...
=======================================================
    SELF-TEST ROUTINE FINISHED. MONITORING READY.     
=======================================================
```

---

## 4. Verification Criteria & Troubleshooting

### Sensor Warning (`WARN: Out of range`)
- If reported voltage is `< 0.15 V`: Potentiometer ground wire is open or sensor signal is shorted to GND.
- If reported voltage is `> 3.15 V`: Potentiometer 3.3V reference is shorted or potentiometer wiper is disconnected (floating high).
- **Correct Value**: When wheels are roughly straight, voltage should be near `1.65 V` ($\approx 32768$ ADC counts).

### Actuator Verification
- Verifies that DAC1 registers accept writes and that `Actuators_ApplySafeState()` resets throttle DAC to `0.0 V`, brake DAC to `0.0 V`, and steering PWM duty to `0%`.

### Comm Verification
- Verifies that `huart3` is active with zero error flags and that DMA is actively cycling.

---

## 5. Python Graphical Test Bench (`aev_diagnostic_gui.py`)

A standalone, dark-themed Python GUI application ([`aev_diagnostic_gui.py`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing/aev_diagnostic_gui.py)) is provided to test all bidirectional communication features from your laptop or Jetson.

### 5.1 Launching the GUI
You can start the GUI in two ways:
- **Option 1 (One-Click Windows)**: Double-click [`run_gui.bat`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing/run_gui.bat).
- **Option 2 (Terminal)**:
  ```bash
  cd App/Testing
  python aev_diagnostic_gui.py
  ```

### 5.2 Futuristic Cockpit Capabilities
1. **Interactive Radial Steering Gauge**:
   - Circular dial ($-30.0^\circ$ to $+30.0^\circ$) displaying **Target Steering Needle** (Neon Cyan) and **Actual EKF Angle Needle** (Neon Green).
   - Click or drag anywhere on the dial to steer interactively!
   - 1-click "Center Wheels (0.0°)" quick button.
2. **Top-Down Vector Vehicle Chassis HUD**:
   - Dynamic 2D geometric wireframe of the Autonomous Electric Vehicle.
   - **Front Wheels rotate dynamically** in real time as steering angle changes!
   - **Rear Drive Wheels illuminate** (Neon Green on acceleration, Neon Red on braking).
3. **50 Hz Rolling Real-Time Oscilloscope**:
   - Multi-trace rolling waveform display plotting Target Angle (Cyan) vs Actual EKF Angle (Green) in real time.
4. **Steering Potentiometer Safety Bounds Meter**:
   - Horizontal sensor bar highlighting the safe operational zone ($0.15\text{V}$ to $3.15\text{V}$) vs endstop fault zones ($<0.15\text{V}$, $>3.15\text{V}$).
5. **Board Hardware Virtual LED Simulators**:
   - Live simulated glowing LEDs at the top of the HUD:
     - 🟢 **`SYS` (PB0)**: Heartbeat pulses.
     - 🟡 **`COMM 10Hz` (PE1)**: Flashes rapidly on data transfer.
     - 🔴 **`FAULT` (PB14)**: Glows on vehicle fault.
6. **Drivetrain & Control Deck**:
   - Modern gear selector pills: `P / NEUTRAL`, `D / FORWARD`, `R / REVERSE`.
   - Accelerator slider ($0\%$ to $100\%$) and braking mode buttons.
   - High-visibility **EMERGENCY STOP (E-STOP)** reactor button.
7. **Serial Packet Radar Console**:
   - Color-coded hex frames + decoded engineering parameters for both TX and RX with auto-scroll.

