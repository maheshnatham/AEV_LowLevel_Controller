# Safety Monitor Architecture (`App/SafetyMonitor`)

## 1. Overview
The **Safety Architecture** enforces fail-safe behavior across all operating modes to prevent uncontrolled acceleration, mechanical over-travel, and runaway motor conditions.

---

## 2. Five-Layer Safety Matrix

| Safety Layer | Trigger Condition | Reaction / Action Taken | Time Constant |
|:---|:---|:---|:---|
| **Layer 1: Power-On Safe State** | MCU Boot / Reset | All DAC outputs initialized to `0.0V`, PWM compare register to `0`, and direction pin low. | $0\text{ ms}$ (Boot time) |
| **Layer 2: Communication Watchdog** | No valid packet received within $200\text{ ms}$ | Immediate throttle cut (`0.0V`), apply safe braking, return steering target to $0.0^\circ$. | $\le 200\text{ ms}$ |
| **Layer 3: Autonomous E-Stop** | Payload field `estop == 1` | Latches emergency safe state; full mechanical brake voltage ($3.3\text{V}$) applied. | $\le 10\text{ ms}$ |
| **Layer 4: Mechanical Jam / Timeout** | Motor stall not detected within $5000\text{ ms}$ during sweep | Enters `STATE_FAULT`, completely cuts motor PWM to protect windings and driver FETs. | $5.0\text{ s}$ |
| **Layer 5: Actuator Saturation Clamping** | High PID error command | Output clamped to safe maximum $\pm 60\%$ effort; integrator clamped to prevent windup. | Instantaneous |

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
if (!link_ok || cmd.estop != 0) {
    Actuators_ApplySafeState();
    jetson_target_angle = 0.0f;
}
```
This guarantees that if the Jetson crashes, ROS freezes, or the serial cable unplugs, the vehicle halts automatically without human intervention.
