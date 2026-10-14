# AEV Sensor Acquisition & Extended Kalman Filter (`App/Sensors`)

## 1. Overview
The **AEV Sensor Acquisition & State Estimation Library** ([`sensors.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/sensors.c), [`sensors.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/sensors.h), [`ekf.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/ekf.c), [`ekf.h`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/Sensors/ekf.h)) acquires feedback from the vehicle's steering rack potentiometer, filters out inverter EMI and motor switching noise, and runs a real-time state estimator:
1. **Hardware ADC DMA Pipeline**: 16-bit analog sampling with continuous Circular DMA oversampling and Exponential Moving Average (EMA) filtering.
2. **Extended Kalman Filter (EKF)**: 2-state recursive estimator predicting true steering position and zero-lag angular velocity directly from physical motor effort.

---

## 2. Hardware ADC Configuration & Filtering

| Parameter | Configuration |
| :--- | :--- |
| Microcontroller Pin | **PA0** (`ADC1_INP16` on Arduino Header **A0**) |
| ADC Resolution | **16-bit** ($0$ to $65535$ counts) |
| DMA Buffer Size | 64 half-words (`ADC_DMA_BUF_SIZE = 64`) |
| Sampling Mode | Continuous Circular DMA (`DMA1_Stream0`) |
| Reference Voltage | $V_{\text{ref}} = 3.3\text{ V}$ |

### 2.1 Two-Stage Noise Rejection Pipeline
1. **Block Oversampling**: During every 10 ms interval, the DMA continuously acquires 64 raw samples. At the 100 Hz tick, `Sensors_Update_100Hz()` averages the 64 samples, eliminating single-sample high-frequency switching noise.
2. **Exponential Moving Average (EMA)**:
   $$\bar{y}_k = \alpha \cdot y_{\text{block}} + (1 - \alpha) \cdot \bar{y}_{k-1}$$
   where $\alpha = 0.20$.
3. **Physical Voltage Conversion**:
   $$V_{\text{pot}} = \left(\frac{\bar{y}_k}{65535.0}\right) \times 3.3\text{ V}$$

---

## 3. Extended Kalman Filter (EKF) State Estimator

### 3.1 State Representation
$$\mathbf{x} = \begin{bmatrix} x_0 \\ x_1 \end{bmatrix} = \begin{bmatrix} \text{Position Voltage } (V) \\ \text{Angular Velocity } (V/s) \end{bmatrix}$$

### 3.2 Why EKF is Used Instead of Finite Difference
Calculating derivative using raw finite differences $\frac{V_k - V_{k-1}}{\Delta t}$ at 100 Hz multiplies electrical noise by a factor of $100$, causing severe jitter in the PID $D$-term. The Kalman filter predicts velocity using a physical motor model and updates it against the measured voltage, yielding clean, zero-lag velocity estimates.

### 3.3 Predict Step (`EKF_Predict`)
Using applied motor effort $u \in [-1.0, +1.0]$ and time step $\Delta t = 0.01\text{ s}$:
$$\hat{x}_0 = x_0 + x_1 \cdot \Delta t$$
$$\hat{x}_1 = x_1 + \left(\frac{u - x_1}{\tau}\right) \cdot \Delta t$$
Covariance $P$ is propagated using process noise covariance $Q$:
$$P = F \cdot P \cdot F^T + Q$$

### 3.4 Update Step (`EKF_Update`)
When new ADC voltage $z$ arrives:
$$\text{Innovation } y = z - \hat{x}_0$$
$$\text{Kalman Gain } K = P \cdot H^T \cdot (H \cdot P \cdot H^T + R)^{-1}$$
$$\mathbf{x} = \mathbf{x} + K \cdot y$$
$$P = (I - K \cdot H) \cdot P$$

### 3.5 Priming the Filter
When the calibration routine finishes, [`app_main.c`](file:///d:/Ph.D%202026-2027/Projects/AEV/STM32/AEV_LowLevel_Controller/AEV_LowLevel_ControllerV1/App/AppCore/app_main.c) calls:
```c
EKF_ResetTo(&steering_ekf, steering_limits.center_v);
```
This initializes $x_0 = V_{\text{center}}$ and $x_1 = 0$, guaranteeing smooth control without transient jolts.
