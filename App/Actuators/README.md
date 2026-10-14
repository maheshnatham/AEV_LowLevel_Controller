# Actuators Library (`App/Actuators`)

## 1. Overview
The **Actuators Library** ([`actuators.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/actuators.c), [`actuators.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/actuators.h)) translates high-level engineering commands (throttle percentage, brake levels, steering effort) into low-level hardware timer registers and digital-to-analog converter (DAC) outputs.

---

## 2. Hardware Mapping & Signals

| Subsystem | Microcontroller Pin | Peripheral Channel | Signal Type | Electrical Range |
|:---|:---|:---|:---|:---|
| **Throttle** | **PA4** | `DAC1_CHANNEL_1` | True Analog Output | 0.0 V to 3.3 V |
| **Brake** | **PA5** | `DAC1_CHANNEL_2` | True Analog Output | 0.0 V to 3.3 V |
| **Steering PWM** | **PC6** | `TIM3_CH1` | PWM Output | 0% to 100% Duty Cycle |
| **Steering DIR** | **PD14** | `GPIO_Output` | Digital Logic Level | 0V (Left) / 3.3V (Right) |

---

## 3. Mathematical Conversions & Driver Logic

### 3.1 Digital-to-Analog Converter (DAC)
STM32H7 features a 12-bit DAC ($2^{12} - 1 = 4095$ counts) with an internal/external $V_{\text{ref}} = 3.3\text{ V}$:
$$\text{DAC Counts} = \left(\frac{V_{\text{target}}}{3.3\text{ V}}\right) \times 4095$$

Clamped strictly between $0.0\text{ V}$ and $3.3\text{ V}$.

### 3.2 Cytron H-Bridge Steering Motor Control
The Cytron DC motor driver takes two control lines: **DIR** (Direction) and **PWM** (Speed):
- **Effort Range**: Normalized from `-1.0` (Full Left) to `+1.0` (Full Right).
- **Direction Pin (PD14)**:
  - If $\text{effort} \ge 0.0$: `HAL_GPIO_WritePin(STEER_DIR_GPIO_Port, STEER_DIR_Pin, GPIO_PIN_SET)`
  - If $\text{effort} < 0.0$: `HAL_GPIO_WritePin(STEER_DIR_GPIO_Port, STEER_DIR_Pin, GPIO_PIN_RESET)`
- **PWM Compare Register**:
  $$\text{CCR1} = |\text{effort}| \times \text{PWM\_MAX\_PERIOD}$$

---

## 4. Failsafe State Enforcement
Calling `Actuators_ApplySafeState()` executes instantly:
1. Sets Throttle DAC to `0.0 V` ($0\text{ counts}$).
2. Sets Brake DAC to `0.0 V` (or emergency brake voltage depending on hardware wiring).
3. Sets Steering PWM compare register to `0` ($0\%\text{ duty cycle}$).

---

## 5. Porting Guide for Different Boards

### Changing PWM Frequency or Timer
In `Core/Src/tim.c`, TIM3 is configured with an Auto-Reload Register (ARR) value.
In `actuators.c`:
```c
// Replace this with your actual Counter Period (ARR) from CubeMX
#define PWM_MAX_PERIOD  13750.0f
```
If you change your timer clock or target PWM frequency (e.g. 20 kHz for silent motor drive), update `PWM_MAX_PERIOD` to match the new `htim3.Init.Period`.

### Changing Pin Allocations
Pin names are abstracted using CubeMX User Labels:
- Label `STEER_DIR` in CubeMX generates `STEER_DIR_Pin` and `STEER_DIR_GPIO_Port` in `main.h`.
- If migrating to a different GPIO pin, simply assign the label `STEER_DIR` to the new pin in `.ioc`.
