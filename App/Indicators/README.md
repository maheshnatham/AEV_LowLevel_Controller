# LED Indicator Library (`App/Indicators`)

## 1. Overview
The **LED Indicator Library** ([`led_indicator.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators/led_indicator.c), [`led_indicator.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Indicators/led_indicator.h)) drives the three onboard user LEDs on the **STM32 NUCLEO-H723ZG** board to provide real-time visual feedback of vehicle state, autonomy serial connection, and diagnostic fault blink codes.

It operates strictly **non-blocking** at 100 Hz inside `App_Step_100Hz()`, consuming zero busy-wait CPU cycles.

---

## 2. Hardware Mapping & STM32CubeMX (.ioc) Configuration

On the **NUCLEO-H723ZG** board, the 3 user LEDs are wired to the following pins:

| Board LED | Color | Microcontroller Pin | CubeMX User Label | Engineering Role |
|:---|:---|:---|:---|:---|
| **LD1** | **Green** | **PB0** | **`SYS_STATUS`** | **System Operating State & Heartbeat** |
| **LD2** | **Yellow** | **PE1** | **`COMM_LINK`** | **Autonomy Serial Link Status (200ms Watchdog)** |
| **LD3** | **Red** | **PB14** | **`SYS_FAULT`** | **Diagnostic Error Blink Codes** |

### Step-by-Step Guide: Adding the Pins in STM32CubeMX
1. Open your project's `.ioc` file in STM32CubeIDE.
2. In the **Pinout & Configuration** tab:
   - Search or click pin **PB0**: Select **`GPIO_Output`**. Right-click $\rightarrow$ **Enter User Label** $\rightarrow$ type **`SYS_STATUS`**.
   - Search or click pin **PE1**: Select **`GPIO_Output`**. Right-click $\rightarrow$ **Enter User Label** $\rightarrow$ type **`COMM_LINK`**.
   - Search or click pin **PB14**: Select **`GPIO_Output`**. Right-click $\rightarrow$ **Enter User Label** $\rightarrow$ type **`SYS_FAULT`**.
3. Under **System Core** $\rightarrow$ **GPIO**:
   - Verify all 3 pins are configured as:
     - **GPIO Mode**: Output Push Pull
     - **GPIO Pull-up/Pull-down**: No pull-up and no pull-down
     - **Maximum output speed**: Low
4. Save the file and click **Generate Code** (Ctrl + S).

*(Note: The code includes safe fallback definitions, so it compiles and functions even before you generate code in CubeMX).*

---

## 3. Visual Signaling Specifications

### 🟢 1. Green LED (LD1 – PB0): System Operating State
| Vehicle Mode | Blink Pattern | Frequency | Visual Meaning |
|:---|:---|:---|:---|
| **`STATE_CALIBRATING`** | Rapid flashing | **5 Hz** (100ms ON / 100ms OFF) | Steering motor is sweeping end-stops — stay clear of wheels. |
| **`STATE_ACTIVE`** | Gentle heartbeat | **1 Hz** (500ms ON / 500ms OFF) | Calibration complete, PID closed-loop active, armed & ready. |
| **`STATE_FAULT` / `INIT`** | Completely OFF | **0 Hz** | Vehicle is not operational. |

---

### 🟡 2. Yellow LED (LD2 – PE1): Autonomy / Serial Link Health & Activity
| Communication Status | LED Behavior | Visual Meaning |
|:---|:---|:---|
| **Active Serial Link (`<= 200 ms`)** | **Rapid Blinking (10 Hz)** | Yellow LED blinks actively (50ms ON / 50ms OFF) whenever serial packets are streaming within the 200ms watchdog. |
| **Single Test Packet** | **1.0-Second Blink Burst** | Flashes actively for 1.0 second upon receiving an individual command frame. |
| **Link Offline / Timeout** | **Completely OFF** | No serial commands received for $> 200\text{ ms}$; failsafe safe-state active. |

---

### 🔴 3. Red LED (LD3 – PB14): Diagnostic Fault "Blink Codes"
When the system is healthy, the Red LED is **completely OFF**.

If an error occurs or the vehicle enters `STATE_FAULT`, the Red LED flashes a repeating count pattern:
> **Format**: $N$ quick pulses ($200\text{ ms}$ ON / $200\text{ ms}$ OFF), followed by a **$1.2\text{-second}$ silent pause**, then repeats.

| Blink Code | Visual Pattern | Error Code Name | Trigger Cause | Action Taken |
|:---:|:---|:---|:---|:---|
| **1 Blink** | `* ______ * ______` | `ERR_CODE_COMM_TIMEOUT` | Host watchdog expired ($> 200\text{ ms}$) while active | Throttle zeroed, partial brake applied, steering centered |
| **2 Blinks** | `* * ______ * * ______` | `ERR_CODE_CALIBRATION_FAIL` | Steering motor did not reach endstop within $5\text{ s}$ or motor power lost | Motor PWM cut, safe state latched |
| **3 Blinks** | `* * * ______ * * * ______` | `ERR_CODE_SENSOR_OUT_OF_BOUNDS` | Steering potentiometer disconnected ($< 0.15\text{ V}$ or $> 3.15\text{ V}$) | Actuators disabled to prevent mechanical over-travel |
| **4 Blinks** | `* * * * ______ * * * * ______` | `ERR_CODE_ESTOP_ACTIVE` | Host Jetson sent packet with `estop == 1` | Throttle cut, emergency brake ($3.3\text{ V}$) latched |
| **Solid ON** | `********************` | `ERR_CODE_FATAL` | Unrecoverable hardware or firmware fault | All outputs shut down immediately |

---

## 4. API Reference

```c
// Initializes Port B and Port E clocks and configures PB0, PE1, PB14
void LedIndicator_Init(void);

// Non-blocking 100 Hz state engine (called from App_Step_100Hz)
void LedIndicator_Update_100Hz(uint8_t system_state, bool comm_active);

// Sets the active diagnostic error code to blink on LD3
void LedIndicator_SetError(ErrorCode_t error);

// Clears active error code and shuts off LD3
void LedIndicator_ClearError(void);

// Queries the active diagnostic code
ErrorCode_t LedIndicator_GetError(void);

// Quick test sequence sweeping Green -> Yellow -> Red
void LedIndicator_TestSequence(void);
```
