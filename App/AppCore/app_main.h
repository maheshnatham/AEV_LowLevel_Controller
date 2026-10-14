#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Master Live Monitoring Struct for STM32CubeIDE Live Expressions.
 *        Organized into clean, intuitive functional groups.
 *        Add `aev_live` to Live Expressions to monitor the entire vehicle in real time,
 *        or inspect individual groups like `aev_live.steering` or `aev_live.drivetrain`.
 */
typedef struct {
    // 1. Host/Jetson High-Level Setpoints & Commands
    struct {
        uint8_t  drive_direction;    // 0 = NEUTRAL, 1 = FORWARD, 2 = REVERSE
        int8_t   steer_direction;    // -1 = LEFT, 0 = CENTER, +1 = RIGHT
        float    steer_angle;        // Target steering angle in degrees
        float    acceleration;       // Requested acceleration % (0.0 to 100.0)
        uint8_t  brake;              // Requested brake mode (0=OFF, 1=ON, 2=EMERGENCY)
        uint8_t  estop;              // Emergency Stop flag (0=Normal, 1=Active)
        uint32_t timestamp_ms;       // Host timestamp from packet
    } command;

    // 2. Closed-Loop Steering & Feedback
    struct {
        float    target_angle_deg;   // Commanded target angle (deg)
        float    actual_angle_deg;   // Measured/estimated angle (deg)
        float    angle_error_deg;    // Error = target - actual (deg)
        float    pot_volts;          // Raw potentiometer sensor voltage (0.0 - 3.3V)
        float    pot_filtered_volts; // EKF estimated position voltage (V)
        uint16_t pot_raw_adc;        // 16-bit ADC raw counts (0 - 65535)
        float    motor_effort;       // PID effort output (-1.00 to +1.00)
        uint8_t  motor_dir_pin;      // Physical DIR GPIO (1 = SET/Right, 0 = RESET/Left)
        float    pwm_duty_pct;       // TIM3 PWM duty cycle percentage (0.0 - 100.0%)
        float    angular_vel_v_s;    // EKF estimated angular velocity (V/s)
    } steering;

    // 3. Drivetrain Throttle & Brake DAC Hardware Outputs
    struct {
        float    throttle_pct;       // Commanded throttle percentage (0.0 - 100.0%)
        float    throttle_volts;     // DAC1 CH1 output voltage (0.0 to 3.3V)
        uint16_t throttle_dac_raw;   // DAC1 CH1 12-bit register value (0 to 4095)
        uint8_t  brake_mode;         // Brake state (0=OFF, 1=PARTIAL, 2=FULL)
        float    brake_volts;        // DAC1 CH2 output voltage (0.0 to 3.3V)
        uint16_t brake_dac_raw;      // DAC1 CH2 12-bit register value (0 to 4095)
    } drivetrain;

    // 4. Calibration Status & Physical Endstop Voltage Map
    struct {
        bool     is_calibrated;      // true once automatic calibration completes
        float    center_volts;       // Potentiometer voltage at physical center
        float    left_limit_volts;   // Potentiometer voltage at left mechanical stop
        float    right_limit_volts;  // Potentiometer voltage at right mechanical stop
        float    degrees_per_volt;   // Scaling factor (deg / V)
    } calibration;

    // 5. System Health, Watchdogs & Fault Diagnostics
    struct {
        uint8_t  system_state;       // 0=INIT, 1=CALIBRATING, 2=ACTIVE, 3=FAULT
        uint8_t  link_alive;         // 1=Link healthy, 0=Failsafe timeout
        uint16_t watchdog_left_ms;   // Time remaining before failsafe trips (ms)
        uint8_t  active_error_code;  // LED fault code (0=NONE, 1=TIMEOUT, 2=CALIB, etc.)
        float    uptime_seconds;     // Uptime in seconds
        uint32_t rx_packet_count;    // Total valid command packets parsed
        uint32_t rx_crc_errors;      // Total corrupted/dropped packets
    } safety;
} VehicleLiveMonitor_t;

extern volatile VehicleLiveMonitor_t aev_live;

void App_Init(void);
void App_Step_100Hz(void);
void App_RequestCalibration(void); // Call from anywhere to re-trigger calibration
uint8_t App_GetSystemState(void);
float App_GetSteeringAngle(void);

#endif /* APP_MAIN_H */
