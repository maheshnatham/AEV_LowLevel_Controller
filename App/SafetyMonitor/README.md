# AEV Multi-Tier Hardware Failsafe & Safety Supervisory Matrix (`App/SafetyMonitor`)

## 1. Overview
The **AEV Safety Architecture** enforces deterministic fail-safe behavior across all operating modes to prevent uncontrolled acceleration, mechanical over-travel, motor driver overcurrent, and runaway steering conditions.

---

## 2. Multi-Layer Safety Matrix

| Safety Layer | Trigger Condition | Reaction / Action Taken | Time Constant |
| :--- | :--- | :--- | :--- |
| **Layer 1: Power-On Safe State** | MCU Boot / Reset | All DAC outputs initialized to `0.0V`, PWM compare register to `0`, direction pin low. | $0\text{ ms}$ (Boot time) |
| **Layer 2: Communication Watchdog** | No valid packet received within $200\text{ ms}$ | Immediate throttle cut (`0.0V`), apply safe partial brake (`1.65V`), relax steering motor to `0.0` effort. | $\le 200\text{ ms}$ |
| **Layer 3: Autonomous E-Stop** | Payload field `estop != 0` | Latches emergency safe state; full emergency brake voltage ($3.3\text{V}$) applied, throttle locked to `0.0V`. | $\le 10\text{ ms}$ (Next tick) |
| **Layer 4: Sensor Integrity Supervision** | Potentiometer voltage $< 0.15\text{V}$ or $> 3.15\text{V}$ | Detects ground disconnect or power short; immediately cuts throttle and flags `ERR_CODE_SENSOR_OUT_OF_BOUNDS`. | $\le 10\text{ ms}$ |
| **Layer 5: Endstop Current Clamping** | Rack position within $40\text{ mV}$ of calibrated end-stop | Clamps pushing effort to `0.0f` to prevent DC motor from drawing locked-rotor stall current ($15\text{A}-30\text{A}+$) against hardstops. | $\le 10\text{ ms}$ |
| **Layer 6: Polarity Auto-Correction** | Reverse-gradient pot or inverted motor wiring | Calibration computes `polarity_sign` ($+1$ or $-1$); PID auto-inverts to mathematically guarantee negative feedback. | Calibration |
| **Layer 7: Actuator Saturation Clamping** | Abrupt step input in angle setpoint | Output clamped to safe maximum $\pm 60\%$ effort; integrator clamped to prevent windup. | Instantaneous |
| **Layer 8: Calibration Sweep Timeout** | Motor stall not confirmed within $10.0\text{ s}$ per sweep | Enters `STATE_FAULT`, completely cuts motor PWM to protect windings and H-bridge FETs. | $10.0\text{ s}$ |

---

## 3. Communication Failsafe Watchdog Details
The communication module continuously increments `system_uptime_ms` and logs `last_valid_msg_time`:
```c
bool Comm_IsLinkActive(void) {
    return (system_uptime_ms - last_valid_msg_time) <= COMM_TIMEOUT_MS; // 200 ms
}
```
In [`app_main.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/app_main.c):
```c
if (!link_ok) {
    current_throttle_v = 0.0f;
    current_brake_v = 1.65f;
    effort = 0.0f; // Relax steering motor
    LedIndicator_SetError(ERR_CODE_COMM_TIMEOUT);
}
```
This guarantees that if the Jetson crashes, ROS freezes, or the serial cable unplugs, the vehicle halts automatically without human intervention.
