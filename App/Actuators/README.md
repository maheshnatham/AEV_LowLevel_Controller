# AEV Actuator Hardware Driver Suite (`App/Actuators`)

## 1. Overview
The **AEV Actuator Hardware Driver Suite** ([`actuators.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/actuators.c), [`actuators.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Actuators/actuators.h)) translates high-level setpoints (throttle percentage, brake levels, steering effort) into physical hardware registers:
- **Dual 12-bit DACs**: Pure analog voltage outputs for traction throttle and friction/regen braking.
- **Hardware Timer PWM (TIM3)**: High-resolution pulse-width modulation for the Cytron DC steering motor.
- **Direction GPIO (PD14)**: High-speed digital logic for H-Bridge directional polarity.

---

## 2. Hardware Mapping & Signals

| Subsystem | Microcontroller Pin | Peripheral Channel | Signal Type | Electrical Range | Hardware Destination |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Throttle** | **PA4** | `DAC1_CHANNEL_1` | True Analog Output | 0.0 V to 3.3 V | Traction Motor Controller |
| **Brake** | **PA5** | `DAC1_CHANNEL_2` | True Analog Output | 0.0 V to 3.3 V | Electric Brake Actuator |
| **Steering PWM** | **PC6** | `TIM3_CH1` | PWM Output | 0% to 100% Duty Cycle | Cytron MDD10A PWM Pin |
| **Steering DIR** | **PD14** | `GPIO_Output` | Digital Logic Level | 0V (Left) / 3.3V (Right) | Cytron MDD10A DIR Pin |

---

## 3. Mathematical Conversions & Driver Logic

### 3.1 Digital-to-Analog Converter (DAC)
The STM32H7 features a dual 12-bit DAC ($2^{12} - 1 = 4095$ counts) referenced to $V_{\text{ref}} = 3.3\text{ V}$:
$$\text{DAC Counts} = \left(\frac{V_{\text{target}}}{3.3\text{ V}}\right) \times 4095$$

Output voltage is clamped strictly between $0.0\text{ V}$ and $3.3\text{ V}$.

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
