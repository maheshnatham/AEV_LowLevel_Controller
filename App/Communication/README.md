# AEV Autonomy Protocol & Telemetry Communication Engine (`App/Communication`)

## 1. Overview
The **Communication Library** manages bidirectional serial communication between the STM32 Low-Level Controller and the high-level autonomy compute (such as an NVIDIA Jetson Orin/Xavier or a laptop running a test terminal like HTerm).

It features:
- **Zero-CPU Overhead Reception**: Utilizes a circular DMA buffer (`DMA1_Stream1`) that streams incoming UART bytes into memory in the background without generating per-byte CPU interrupts.
- **Robust Binary Framing**: Strict Start-of-Frame (`0xAA 0x55`) and End-of-Frame (`0x55 0xAA`) delimiters with length verification and CRC-16 validation.
- **Failsafe Watchdog (200 ms)**: If a valid control packet is not received within `COMM_TIMEOUT_MS` (200 ms), the link is declared dead and the vehicle disarms throttle and engages safety brakes.
- **50 Hz Telemetry Transmission**: Periodically broadcasts vehicle state, actual steering angle, potentiometer volts, applied actuator efforts, and watchdog heartbeat back to the host computer.

---

## 2. Packet Framing & Protocol Specification

All frames follow this binary envelope:

```
+-----------+--------+--------+--------+------------------+---------------+-----------+
| SOF (2B)  | MSG_ID | SEQ    | LEN    | PAYLOAD (N Bytes)| CRC-16 (2B)   | EOF (2B)  |
| 0xAA 0x55 | 1 Byte | 1 Byte | 1 Byte | Packed Structure | Little-Endian | 0x55 0xAA |
+-----------+--------+--------+--------+------------------+---------------+-----------+
```

### 2.1 CRC-16 Specification
- **Algorithm**: CRC-16-CCITT
- **Polynomial**: `0x1021` ($x^{16} + x^{12} + x^5 + 1$)
- **Initial Value**: `0xFFFF`
- **Data Coverage**: Calculated over `[MSG_ID, SEQ, LEN, PAYLOAD]` (from byte offset 2 up to the end of payload).
- **Endianness in Frame**: Sent as `[CRC_Low_Byte, CRC_High_Byte]`.

---

## 3. Message Definitions

### 3.1 Control Command Packet (`MSG_ID_CMD_CONTROL = 0x01`)
Sent from **Jetson / Laptop -> STM32**:

| Field Offset | Field Name | Type | Description |
|:---|:---|:---|:---|
| 0 | `drive_direction` | `uint8_t` | `0` = Neutral, `1` = Forward, `2` = Reverse |
| 1 | `steer_direction` | `int8_t`  | `-1` = Left, `0` = Center, `+1` = Right |
| 2..5 | `steer_angle` | `float` (32-bit IEEE 754) | Target wheel angle in degrees (e.g. `0.0f`, `15.0f`, `-15.0f`) |
| 6..9 | `acceleration` | `float` (32-bit IEEE 754) | Throttle percentage: `0.0` to `100.0%` |
| 10 | `brake` | `uint8_t` | `0` = Off, `1` = Partial Brake, `2` = Emergency Brake |
| 11 | `estop` | `uint8_t` | `0` = Normal Run, `1` = Emergency Stop (Immediate Safe State) |
| 12..15 | `timestamp_ms` | `uint32_t` | Host millisecond timestamp |

**Total Payload Size**: 16 Bytes.

---

### 3.2 Telemetry Status Packet (`MSG_ID_TLM_STATUS = 0x81`)
Sent from **STM32 -> Jetson / Laptop**:

| Field Offset | Field Name | Type | Description |
|:---|:---|:---|:---|
| 0 | `system_state` | `uint8_t` | `0` = INIT, `1` = CALIBRATING, `2` = ACTIVE, `3` = FAULT |
| 1..4 | `actual_steer_angle` | `float` | Current estimated steering angle in degrees |
| 5..8 | `steering_pot_v` | `float` | Current steering potentiometer voltage (0.0 to 3.3V) |
| 9..12 | `applied_steer_effort` | `float` | Motor PWM effort (-1.0 to +1.0) |
| 13..16 | `applied_throttle_v` | `float` | Throttle DAC output voltage (0.0 to 3.3V) |
| 17..20 | `applied_brake_v` | `float` | Brake DAC output voltage (0.0 to 3.3V) |
| 21 | `link_alive` | `uint8_t` | `1` = Link healthy, `0` = Watchdog timeout active |
| 22..25 | `uptime_ms` | `uint32_t` | STM32 system uptime in milliseconds |

**Total Payload Size**: 26 Bytes.

---

## 4. How to Test with HTerm (Laptop Quick-Start)

### 4.1 Connection Setup
1. Connect the STM32 NUCLEO board to your laptop using the ST-LINK USB cable.
2. Open **HTerm** (or your serial terminal of choice).
3. Select the COM port associated with **STMicroelectronics STLink Virtual COM Port**.
4. Configure:
   - **Baudrate**: `115200`
   - **Data bits**: `8`
   - **Stop bits**: `1`
   - **Parity**: `None`
   - **Flow control**: `None`
5. Click **Connect**.

### 4.2 Receiving Telemetry in HTerm
- In HTerm's **Received Data** pane, select **HEX** view.
- Once calibration completes and the system enters `STATE_ACTIVE`, you will see recurring frames starting with `AA 55 81 ... 55 AA`.

### 4.3 Sending Control Commands from HTerm
In HTerm, switch the **Input control** to **HEX** mode. You can copy and paste the following verified hex packets:

#### Command 1: Heartbeat / Neutral Center (Steer 0.0°, Throttle 0%, E-Stop 0)
```hex
AA 55 01 01 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4E 83 55 AA
```

#### Command 2: Steer 10.0° Right, Drive Forward 15% Throttle
```hex
AA 55 01 02 10 01 01 00 00 20 41 00 00 70 41 00 00 00 00 00 00 59 A2 55 AA
```

#### Command 3: Immediate Emergency Stop (`estop = 1`, `brake = 2`)
```hex
AA 55 01 03 10 00 00 00 00 00 00 00 00 00 00 02 01 00 00 00 00 1C 52 55 AA
```

---

## 5. Board Porting Guide: How to Change the UART Peripheral

By default on the **NUCLEO-H723ZG**, the communication module uses **USART3** because pins **PD8 (TX)** and **PD9 (RX)** connect directly to the ST-LINK Virtual COM port over the USB cable.

When migrating to a custom board or another UART instance (e.g., `USART1` or `USART2` connected directly to Jetson GPIO headers):

### Step 1: STM32CubeMX Configuration
1. In the `.ioc` pinout view, enable your chosen peripheral (e.g., **USART1** or **USART2**) in **Asynchronous** mode.
2. Under **Parameter Settings**:
   - Baud Rate: `115200`
   - Word Length: `8 Bits`
   - Parity: `None`
   - Stop Bits: `1`
3. Under **DMA Settings**:
   - Add DMA Request: `USARTx_RX`
   - Mode: **Circular**
   - Increment Address: Peripheral = `Disable`, Memory = `Enable`
   - Data Width: Peripheral = `Byte`, Memory = `Byte`
4. Under **NVIC Settings**:
   - Enable `USARTx global interrupt` and `DMAx stream interrupt`.
5. Click **Generate Code**.

### Step 2: Code Adjustments in `protocol.c`
In [`protocol.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Communication/protocol.c):
1. Change the extern reference from `huart3` to your new handle (e.g. `huart1` or `huart2`):
   ```c
   // If using USART1 instead of USART3:
   extern UART_HandleTypeDef huart1;
   #define COMM_UART huart1
   ```
2. Replace all occurrences:
   - In `Comm_Init()`: `HAL_UART_Receive_DMA(&COMM_UART, rx_dma_buf, RX_DMA_BUF_SIZE);`
   - In `Comm_Update_100Hz()`: `__HAL_DMA_GET_COUNTER(COMM_UART.hdmarx)`
   - In `Comm_SendTelemetry()`: `HAL_UART_Transmit_IT(&COMM_UART, tx_frame_buf, idx);`
   - In `Comm_SendString()`: `HAL_UART_Transmit(&COMM_UART, ...);`

---

## 6. Failsafe Logic & Edge Case Protections

1. **Circular DMA Pointer Wrap**:
   The calculation `dma_write_ptr = (RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx)) % RX_DMA_BUF_SIZE;` ensures that when `NDTR == 0` (instant of buffer wraparound), `dma_write_ptr` is properly clamped to `0`, preventing out-of-bounds array reads.
2. **Buffer Overrun Protection**:
   `tx_frame_buf` is declared `static` to ensure valid lifetime during non-blocking interrupt transmission (`HAL_UART_Transmit_IT`). The function also checks `huart3.gState == HAL_UART_STATE_READY` before issuing a transmit, eliminating packet interleaving or driver locking.
