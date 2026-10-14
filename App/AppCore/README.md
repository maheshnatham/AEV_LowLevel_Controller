# Application Core Library (`App/AppCore`)

## 1. Overview
The **Application Core** ([`app_main.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/app_main.c), [`app_main.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/app_main.h)) is the central state machine and supervisory scheduler of the Autonomous Electric Vehicle (AEV) controller.

It bridges all individual libraries (`Sensors`, `Actuators`, `MotionControl`, `Communication`) into a deterministic, hard real-time 100 Hz control loop.

---

## 2. System State Machine

```
   [Boot / Reset]
         │
         ▼
   ┌────────────┐
   │ STATE_INIT │  (Sensors, Actuators, Calibration, Comm, PID, EKF initialized)
   └─────┬──────┘
         │
         ▼
 ┌───────────────────┐               Calibration Fault
 │ STATE_CALIBRATING │ ───────────────────────────────────────┐
 └───────┬───────────┘                                        │
         │ Calibration Done                                   │
         ▼                                                    ▼
   ┌──────────────┐           Watchdog Timeout (>200ms)    ┌─────────────┐
   │ STATE_ACTIVE │ ─────────────────────────────────────► │ STATE_FAULT │
   └──────┬───────┘           or Hardware Critical Fault   └─────────────┘
          ▲
          │ User Button (PC13) / Recalibration Request
          └───────────────────────────────────────── (Safe Stop -> Reset to STATE_CALIBRATING)
```

### State Descriptions:
1. **`STATE_INIT`**:
   - Initialises ADC DMA, DAC channels, PWM timer, UART DMA receiver, PID gains, and Kalman filter covariance matrices.
   - Automatically transitions to `STATE_CALIBRATING`.
2. **`STATE_CALIBRATING`**:
   - The steering motor sweeps to physical end-stops to establish voltage-to-angle conversion factors and mechanical center.
   - **Isolation**: Autonomy commands from Jetson/HTerm are safely ignored so calibration cannot be disturbed.
3. **`STATE_ACTIVE`**:
   - Closed-loop PID position control is active.
   - **EKF** estimates true steering position and velocity.
   - **Watchdog Check**: Validates incoming packets within 200 ms. If the link fails or E-Stop is active, vehicle executes emergency stop.
   - **Telemetry Dispatch**: Broadcasts vehicle telemetry back to host computer at 50 Hz.
4. **`STATE_FAULT`**:
   - Shuts off motor PWM, sets throttle DAC to 0.0V, and engages full mechanical brakes.

---

## 3. Timing Architecture & Interrupt Handlers

### 3.1 100 Hz Deterministic Loop
- Driven by **Hardware Timer TIM6** configured to generate an update interrupt every **10.0 ms** (100 Hz).
- Inside `Core/Src/main.c`:
  ```c
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
      if (htim->Instance == TIM6) {
          App_Step_100Hz();
      }
  }
  ```

### 3.2 Thread-Safe Recalibration Mechanism
- Triggered on-demand via the Nucleo User Button (PC13 `Motor_Calib`), debugger (`recal_from_debugger = 1`), or UART command.
- Uses a deferred handshake flag:
  ```c
  void App_RequestCalibration(void) {
      recal_requested = true;
  }
  ```
- Checked at the very top of `App_Step_100Hz()` before any calculations run:
  1. Clears flag.
  2. Calls `Actuators_ApplySafeState()` immediately.
  3. Resets calibration state machine.
  4. Returns to start fresh on the subsequent tick.

---

## 4. Hardware Customization & Board Porting

| Parameter | Default (Nucleo-H723ZG) | How to Change |
|:---|:---|:---|
| Loop Timer | `TIM6` (100 Hz) | Modify timer in CubeMX; update callback check in `main.c`. |
| User Button Pin | `PC13` (`Motor_Calib`) | Change EXTI pin label in CubeMX; update `HAL_GPIO_EXTI_Callback` in `main.c`. |
| Loop Rate | 100 Hz (10 ms) | Update TIM6 prescaler/period; update `dt = 0.01f` in PID & EKF calls. |

---

## 5. Live Expressions Real-Time Monitoring (`aev_live`)

To inspect all vehicle operations during debugging without breakpoints, add `aev_live` to the **Live Expressions** tab in STM32CubeIDE.

```c
extern volatile VehicleLiveMonitor_t aev_live;
```

It is refreshed synchronously at 100 Hz in `App_Step_100Hz()` and broken down into 5 intuitive functional sub-groups:

| Category | Expression Name | Description |
|:---|:---|:---|
| **Host Command** | `aev_live.command` | Latest setpoints received (`drive_direction`, `steer_angle`, `acceleration`, `brake`, `estop`, `timestamp_ms`). |
| **Steering Control** | `aev_live.steering` | Closed-loop PID & feedback (`target_angle_deg`, `actual_angle_deg`, `angle_error_deg`, `pot_volts`, `pot_filtered_volts`, `pot_raw_adc`, `motor_effort`, `motor_dir_pin`, `pwm_duty_pct`, `angular_vel_v_s`). |
| **Drivetrain Hardware** | `aev_live.drivetrain` | Actuator outputs (`throttle_pct`, `throttle_volts`, `throttle_dac_raw` [0-4095], `brake_mode`, `brake_volts`, `brake_dac_raw` [0-4095]). |
| **Calibration Limits** | `aev_live.calibration` | Physical endstop map (`is_calibrated`, `center_volts`, `left_limit_volts`, `right_limit_volts`, `degrees_per_volt`). |
| **Safety & Watchdogs** | `aev_live.safety` | Link and diagnostic health (`system_state`, `link_alive`, `watchdog_left_ms`, `active_error_code`, `uptime_seconds`, `rx_packet_count`, `rx_crc_errors`). |

> [!TIP]
> You can also directly add individual sub-groups such as `aev_live.steering` or `aev_live.drivetrain` into Live Expressions to keep your watch window focused.

