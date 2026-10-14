# Motion Control Library (`App/MotionControl`)

## 1. Overview
The **Motion Control Library** contains algorithms for:
1. **Automated Steering Calibration** ([`calibration.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl/calibration.c)): Sweeps steering wheels across physical end-stops to automatically discover potentiometer voltages, calculate the true mechanical center, and compute the voltage-to-degrees scaling factor.
2. **Precision Closed-Loop Steering PID** ([`pid.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl/pid.c)): Drives the Cytron steering motor using velocity feedback from the Extended Kalman Filter (EKF) with anti-windup clamping.

---

## 2. Steering Calibration State Machine

```
  CAL_STATE_START
        │
        ▼
  CAL_STATE_SWEEP_DIR_1 ──(Stall detected at End-Stop 1)──► CAL_STATE_BACKOFF
                                                                  │
                                                                  ▼ (Backs off 500ms)
  CAL_STATE_RETURN_CENTER ◄──(Stall detected at End-Stop 2)── CAL_STATE_SWEEP_DIR_2
        │
        ▼ (Drives to center_v)
  CAL_STATE_DONE
```

### 2.1 Stall Detection Mechanism
The algorithm determines that the steering mechanism has hit an end-stop without requiring limit switches:
1. **Effort Applied**: Drives motor at a safe, low effort of $\pm 30\%$ (`CAL_SWEEP_EFFORT = 0.30f`).
2. **Voltage Delta Check**:
   $$\Delta V = |V_{\text{current}} - V_{\text{previous}}|$$
   If $\Delta V < 20\text{ mV}$ (`CAL_STALL_VOLT_DELTA = 0.020f`), the potentiometer is stationary.
3. **Settling & Confirmation Timers**:
   - **0 to 200 ms**: Motor startup settling window — stall detection is bypassed to allow the motor to overcome static inertia.
   - **Reset Condition**: As long as the motor is actively turning ($\Delta V \ge 20\text{ mV}$), the timer is held at `200 ms`.
   - **Confirmation**: The moment the motor hits the hard physical stop, $\Delta V$ drops below $20\text{ mV}$. The timer counts from `200 ms` to `300 ms` ($100\text{ ms}$ confirmation window) to officially record the end-stop voltage.

### 2.2 Calibration Output Parameters
Once both end-stops are found:
- **Center Voltage**:
  $$V_{\text{center}} = \frac{V_{\text{left}} + V_{\text{right}}}{2}$$
- **Scale Factor**:
  $$\text{Scale} = \frac{\text{TOTAL\_PHYSICAL\_RANGE\_DEG}}{|V_{\text{right}} - V_{\text{left}}|} \quad [\text{degrees per volt}]$$
  *(Default full physical steering lock-to-lock range is $70.0^\circ$)*.

---

## 3. Closed-Loop PID Controller

### 3.1 Derivative-on-Measurement Architecture
Traditional PID calculates derivative on error: $\frac{d(e)}{dt} = \frac{d(r - y)}{dt}$. When the target angle changes abruptly, this causes large current spikes ("derivative kick").

Our controller calculates derivative purely on the **measured velocity** provided by the Extended Kalman Filter:
$$u(t) = K_p \cdot e(t) + K_i \int e(t)\,dt - K_d \cdot \dot{y}_{\text{EKF}}(t)$$

### 3.2 Anti-Windup Clamping
The integral accumulator is strictly clamped within $[-\text{integrator\_limit}, +\text{integrator\_limit}]$:
```c
if (pid->integrator > pid->integrator_limit) pid->integrator = pid->integrator_limit;
if (pid->integrator < -pid->integrator_limit) pid->integrator = -pid->integrator_limit;
```

### 3.3 Default Gains & Limits
```c
PID_Init(&steering_pid, 
         5.0f,    // Kp (Proportional Gain)
         1.0f,    // Ki (Integral Gain)
         0.20f,   // Kd (Derivative Gain)
         -0.60f,  // Output Min (-60% PWM effort limit)
         0.60f,   // Output Max (+60% PWM effort limit)
         0.25f);  // Integrator Saturation Limit
```

---

## 4. Tuning Guidelines for Different Vehicles
- **Faster Steering Response**: Increase $K_p$ in steps of 0.5.
- **Oscillations / Overshoot around Center**: Increase $K_d$ in steps of 0.05 to add damping.
- **Steady-State Angle Offset Under Load**: Increase $K_i$ in steps of 0.2.
- **Max Effort Safety**: Adjust `out_min` / `out_max` to prevent over-torquing the steering linkages.
