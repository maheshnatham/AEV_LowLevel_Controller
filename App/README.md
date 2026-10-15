# AEV Low-Level Controller Software Architecture (`App/`)

## 1. High-Level Architecture Overview

The `App/` directory contains all custom, board-independent application and algorithm libraries for the Autonomous Electric Vehicle (AEV) Low-Level Controller.

```
                       ┌─────────────────────────┐
                       │  NVIDIA Jetson / Laptop │
                       │ (ROS 2 / Diagnostic GUI)│
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
  │  (ADC DMA, EKF)   │    │(Auto-Calib, PID)  │    │(DAC Throttle/Brk, │
  │                   │    │                   │    │ TIM3 Steering PWM)│
  └───────────────────┘    └───────────────────┘    └───────────────────┘
```

---

## 2. Directory Structure & Subsystem Documentation

| Module Directory | Primary Source Files | Dedicated Manual | Engineering Purpose |
|:---|:---|:---|:---|
| [`AppCore/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore) | `app_main.c`, `app_main.h` | [AppCore README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/AppCore_README.md) | Central 100 Hz real-time executive, state machine, and `aev_live` monitor. |
| [`MotionControl/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl) | `calibration.c`, `pid.c` | [MotionControl README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/MotionControl/Motion_README.md) | Auto-polarity calibration, soft end-stop clamping, and EKF-damped PID steering. |
| [`Actuators/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators) | `actuators.c`, `actuators.h` | [Actuators README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/README.md) | Dual 12-bit DAC drivers (PA4/PA5) and TIM3 steering PWM compare register control. |
| [`Sensors/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors) | `sensors.c`, `ekf.c` | [Sensors README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/Sensor_README.md) | 16-bit ADC circular DMA oversampling, EMA filtering, and 2-state Kalman filter. |
| [`Communication/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Communication) | `protocol.c`, `protocol.h` | [Communication README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Communication/Comm_README.md) | Circular DMA UART protocol, CRC-16 CCITT validation, 50 Hz telemetry, and 200 ms watchdog. |
| [`SafetyMonitor/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/SafetyMonitor) | Architecture docs | [SafetyMonitor README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/SafetyMonitor/Safety_README.md) | Multi-tier hardware failsafe matrix, E-Stop latching, and sensor bounds checking. |
| [`Indicators/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators) | `led_indicator.c`, `led_indicator.h` | [Indicators README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators/Led_README.md) | Non-blocking visual status feedback (LD1 State, LD2 Comm link, LD3 Error blink codes). |
| [`Testing/`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing) | `system_test.c`, `aev_diagnostic_gui.py` | [Testing README](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Testing/README.md) | Ground station testing GUI, pre-flight ASCII diagnostics, and HTerm routines. |

---

## 3. Quick-Start Guide for Testing

### 1. Graphical Testing with Python Ground Station
Launch the interactive diagnostic GUI:
```powershell
python App\Testing\aev_diagnostic_gui.py
```
- Connect to the ST-Link COM port (`115200 8N1`).
- Click **"Start 50 Hz Streaming"** to keep the watchdog active.
- Move the steering angle or throttle slider to test real-time response.

### 2. Recalibration on Demand
- Press the onboard blue User Button (**B1 - PC13 `Motor_Calib`**).
- The steering sweeps right, backs off, sweeps left, finds center, identifies polarity, and resets the Kalman filter.

### 3. Monitoring via STM32CubeIDE Live Expressions
- Add `aev_live` to the **Live Expressions** tab to observe all commands, feedback, DAC outputs, and error states live at 100 Hz.
