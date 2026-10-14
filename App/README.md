# AEV Low-Level Controller Software Architecture (`App/`)

## 1. High-Level Architecture Overview

The `App/` directory contains all custom, board-independent application and algorithm libraries for the Autonomous Electric Vehicle (AEV) Low-Level Controller.

```
                       ┌─────────────────────────┐
                       │  NVIDIA Jetson / Laptop │
                       └────────────┬────────────┘
                                    │ UART (115200 8N1)
                                    ▼
                       ┌─────────────────────────┐
                       │    App/Communication    │ (protocol.c / protocol.h)
                       └────────────┬────────────┘
                                    │ Packets / Watchdog
                                    ▼
                       ┌─────────────────────────┐
                       │       App/AppCore       │ (app_main.c / app_main.h)
                       │  100 Hz State Machine   │
                       └───┬─────────┬─────────┬─┘
                           │         │         │
             ┌─────────────┘         │         └──────────────┐
             ▼                       ▼                        ▼
  ┌───────────────────┐    ┌───────────────────┐    ┌───────────────────┐
  │    App/Sensors    │    │ App/MotionControl │    │   App/Actuators   │
  │  (ADC DMA, EKF)   │    │ (Calibration, PID)│    │(DAC Throttle/Brk, │
  │                   │    │                   │    │ TIM3 Steering PWM)│
  └───────────────────┘    └───────────────────┘    └───────────────────┘
```

---

## 2. Directory Structure & Library Documentation

| Directory | Primary Files | Documentation Link | Purpose |
|:---|:---|:---|:---|
| [`AppCore/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore) | `app_main.c`, `app_main.h` | [AppCore README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/README.md) | Central 100 Hz real-time scheduler and state machine. |
| [`Communication/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Communication) | `protocol.c`, `protocol.h` | [Communication README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Communication/README.md) | Circular DMA UART protocol, CRC-16, telemetry, and 200ms watchdog. |
| [`MotionControl/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl) | `calibration.c`, `pid.c` | [MotionControl README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl/README.md) | Auto-calibration with stall detection and EKF velocity-based PID. |
| [`Actuators/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators) | `actuators.c`, `actuators.h` | [Actuators README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/README.md) | Hardware drivers for DAC1 (Throttle/Brake) and TIM3 (Steering PWM). |
| [`Sensors/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors) | `sensors.c`, `ekf.c` | [Sensors README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/README.md) | 16-bit ADC oversampling, EMA filtering, and 2-state Kalman filter. |
| [`SafetyMonitor/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/SafetyMonitor) | Architecture docs | [SafetyMonitor README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/SafetyMonitor/README.md) | 5-layer safety matrix and failsafe protocols. |
| [`Indicators/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators) | `led_indicator.c`, `led_indicator.h` | [Indicators README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators/README.md) | Visual feedback (LD1 State, LD2 Comm link, LD3 Error blink codes). |
| [`Testing/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing) | `system_test.c`, `system_test.h` | [Testing README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing/README.md) | Pre-flight diagnostics and HTerm testing routines. |

---

## 3. Quick-Start Guide for Testing

### 1. Initial Hardware Check with HTerm
1. Connect Nucleo board via ST-LINK USB to laptop.
2. Open **HTerm** at `115200 8N1`.
3. In STM32CubeIDE **Live Expressions**, set `run_system_test = 1`.
4. HTerm displays the comprehensive diagnostic report and sends a test binary telemetry frame.

### 2. Recalibration on Demand
- Press the onboard blue User Button (**B1 - PC13 `Motor_Calib`**).
- Alternatively, set `recal_from_debugger = 1` in Live Expressions.
- The steering sweeps right, backs off, sweeps left, finds center, and resets the Kalman filter.

### 3. Running Autonomy / Remote Control
- Send binary command packets (MsgID `0x01`) at 50 Hz - 100 Hz from your Jetson or HTerm.
- Verify vehicle replies with Telemetry frames (MsgID `0x81`).
