#include "app_main.h"
#include "main.h"
#include "sensors.h"
#include "ekf.h"
#include "calibration.h"
#include "pid.h"
#include "actuators.h"
#include "protocol.h"
#include "led_indicator.h"

typedef enum {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_ACTIVE,
    STATE_FAULT
} SystemState_t;

static SystemState_t current_state = STATE_INIT;
static EKF_t steering_ekf;
static PID_t steering_pid;
static CalData_t steering_limits;

// Target steering angle in degrees — populated from Jetson/HTerm via UART (protocol.c)
static float jetson_target_angle = 0.0f;
static float current_throttle_v = 0.0f;
static float current_brake_v = 0.0f;

// Recalibration request flag — set this from anywhere (button ISR, debugger, UART)
// volatile ensures the compiler does not optimise away reads of this flag
static volatile bool recal_requested = false;

// Master Live Monitoring Struct for STM32CubeIDE Live Expressions
volatile VehicleLiveMonitor_t aev_live = {0};



void App_Init(void) {
    Sensors_Init();
    Actuators_Init();
    Calibration_Init();
    Comm_Init(); // Start UART DMA circular reception
    LedIndicator_Init(); // Initialize onboard LEDs (Green, Yellow, Red)

    // Initialize PID (Kp, Ki, Kd, Output Min, Output Max, Integrator Limit)
    PID_Init(&steering_pid, 5.0f, 1.0f, 0.20f, -0.60f, 0.60f, 0.25f);

    // Initialize EKF with default guesses until calibration finishes
    EKF_Init(&steering_ekf, 1.65f, 10.0f, 0.005f);

    current_state = STATE_CALIBRATING;
}

// ---------------------------------------------------------------------------
// App_RequestCalibration
// Safe to call from: button GPIO ISR, debugger, UART command handler.
// The actual reset is deferred and executed inside App_Step_100Hz so that
// it always happens in the same IRQ context as all hardware writes.
// ---------------------------------------------------------------------------
void App_RequestCalibration(void) {
    recal_requested = true;
}

uint8_t App_GetSystemState(void) {
    return (uint8_t)current_state;
}

float App_GetSteeringAngle(void) {
    if (steering_limits.degrees_per_volt > 0.001f) {
        return (steering_ekf.x[0] - steering_limits.center_v) * steering_limits.degrees_per_volt * (float)steering_limits.polarity_sign;
    }
    return 0.0f;
}

static void App_UpdateLiveMonitor(ControlPayload_t cmd, bool link_ok, float raw_steering_v) {
    // 1. Host Command & Setpoints
    aev_live.command.drive_direction = cmd.drive_direction;
    aev_live.command.steer_direction = cmd.steer_direction;
    aev_live.command.steer_angle     = cmd.steer_angle;
    aev_live.command.acceleration    = cmd.acceleration;
    aev_live.command.brake           = cmd.brake;
    aev_live.command.estop           = cmd.estop;
    aev_live.command.timestamp_ms    = cmd.timestamp_ms;

    // 2. Closed-Loop Steering & Feedback
    float actual_angle = App_GetSteeringAngle();
    aev_live.steering.target_angle_deg   = jetson_target_angle;
    aev_live.steering.actual_angle_deg   = actual_angle;
    aev_live.steering.angle_error_deg    = jetson_target_angle - actual_angle;
    aev_live.steering.pot_volts          = raw_steering_v;
    aev_live.steering.pot_filtered_volts = steering_ekf.x[0];
    aev_live.steering.pot_raw_adc        = (uint16_t)Sensors_GetSteeringRawAvg();
    aev_live.steering.motor_effort       = Actuators_GetLastSteeringEffort();
    aev_live.steering.motor_dir_pin      = Actuators_GetSteerDirPin();
    aev_live.steering.pwm_duty_pct       = Actuators_GetSteerPwmDutyPct();
    aev_live.steering.angular_vel_v_s    = steering_ekf.x[1];

    // 3. Drivetrain Throttle & Brake DAC Hardware Outputs
    aev_live.drivetrain.throttle_pct     = cmd.acceleration;
    aev_live.drivetrain.throttle_volts   = current_throttle_v;
    aev_live.drivetrain.throttle_dac_raw = (uint16_t)Actuators_GetThrottleDacCounts();
    aev_live.drivetrain.brake_mode       = cmd.brake;
    aev_live.drivetrain.brake_volts      = current_brake_v;
    aev_live.drivetrain.brake_dac_raw    = (uint16_t)Actuators_GetBrakeDacCounts();

    // 4. Calibration Status & Physical Endstops
    aev_live.calibration.is_calibrated     = (current_state == STATE_ACTIVE);
    aev_live.calibration.center_volts      = steering_limits.center_v;
    aev_live.calibration.left_limit_volts  = steering_limits.limit_low_v;
    aev_live.calibration.right_limit_volts = steering_limits.limit_high_v;
    aev_live.calibration.degrees_per_volt  = steering_limits.degrees_per_volt;

    // 5. System Health, Watchdogs & Fault Diagnostics
    aev_live.safety.system_state      = (uint8_t)current_state;
    aev_live.safety.link_alive        = link_ok ? 1 : 0;
    aev_live.safety.watchdog_left_ms  = (uint16_t)Comm_GetWatchdogRemainingMs();
    aev_live.safety.active_error_code = (uint8_t)LedIndicator_GetError();
    aev_live.safety.uptime_seconds    = (float)HAL_GetTick() / 1000.0f;
    aev_live.safety.rx_packet_count   = comm_rx_packets;
    aev_live.safety.rx_crc_errors     = comm_rx_crc_errs;
}

void App_Step_100Hz(void) {
    // --- Recalibration Request Handler -----------------------------------
    // Processed first so the system is safe before anything else runs.
    if (recal_requested) {
        recal_requested = false;
        Actuators_ApplySafeState();   // Zero all motor outputs immediately
        Calibration_Init();           // Reset the calibration state machine
        current_state = STATE_CALIBRATING;
        return;                       // Skip this tick — start fresh next tick
    }

    // 1. Process UART reception circular DMA buffer
    Comm_Update_100Hz();
    bool link_ok = Comm_IsLinkActive();
    ControlPayload_t cmd = Comm_GetLatestCommand();

    // 2. Read latest ADC average from DMA buffer
    Sensors_Update_100Hz();
    float raw_steering_v = Sensors_GetSteeringVolts();

    switch (current_state) {

        case STATE_CALIBRATING:
        {
            float cal_motor_effort = 0.0f;
            CalState_t cal_status = Calibration_Step_100Hz(raw_steering_v, &cal_motor_effort);

            if (cal_status == CAL_STATE_DONE) {
                // Calibration complete — store results and prime EKF at center
                steering_limits = Calibration_GetResults();
                EKF_ResetTo(&steering_ekf, steering_limits.center_v);
                current_state = STATE_ACTIVE;
                LedIndicator_ClearError();
            }
            else if (cal_status == CAL_STATE_FAULT) {
                current_state = STATE_FAULT;
                LedIndicator_SetError(ERR_CODE_CALIBRATION_FAIL);
            }
            else {
                // Drive motor with effort computed by calibration state machine
                Actuators_SetSteeringEffort(cal_motor_effort);
            }
            break;
        }

        case STATE_ACTIVE:
        {
            // --- Failsafe Watchdog & E-Stop Monitoring ---
            if (cmd.estop != 0) {
                current_throttle_v = 0.0f;
                current_brake_v = 3.3f; // Full emergency brake on estop
                Actuators_SetThrottle_Volts(current_throttle_v);
                Actuators_SetBrake_Volts(current_brake_v);
                jetson_target_angle = 0.0f;
                LedIndicator_SetError(ERR_CODE_ESTOP_ACTIVE);
            } else if (!link_ok) {
                current_throttle_v = 0.0f;
                current_brake_v = 1.65f;
                Actuators_SetThrottle_Volts(current_throttle_v);
                Actuators_SetBrake_Volts(current_brake_v);
                jetson_target_angle = 0.0f;
                LedIndicator_SetError(ERR_CODE_COMM_TIMEOUT);
            } else if (raw_steering_v < 0.15f || raw_steering_v > 3.15f) {
                // Potentiometer sensor disconnection/short
                current_throttle_v = 0.0f;
                Actuators_SetThrottle_Volts(current_throttle_v);
                LedIndicator_SetError(ERR_CODE_SENSOR_OUT_OF_BOUNDS);
            } else {
                // Link is healthy and sensors nominal
                LedIndicator_ClearError();
                jetson_target_angle = cmd.steer_angle;

                // Throttle: allow in FORWARD (1), REVERSE (2), or whenever acceleration > 0%
                if (cmd.drive_direction == 1 || cmd.drive_direction == 2 || cmd.acceleration > 0.0f) {
                    current_throttle_v = (cmd.acceleration / 100.0f) * 3.3f;
                    if (current_throttle_v > 3.3f) current_throttle_v = 3.3f;
                    if (current_throttle_v < 0.0f) current_throttle_v = 0.0f;
                } else {
                    current_throttle_v = 0.0f; // Neutral with 0% acceleration
                }
                Actuators_SetThrottle_Volts(current_throttle_v);

                // Brake: 0=OFF, 1=ON (partial), 2=EMERGENCY
                if (cmd.brake == 1) {
                    current_brake_v = 1.65f;
                } else if (cmd.brake == 2) {
                    current_brake_v = 3.3f;
                } else {
                    current_brake_v = 0.0f;
                }
                Actuators_SetBrake_Volts(current_brake_v);
            }

            // Closed-loop steering control
            // 1. Predict next state using last known motor effort
            EKF_Predict(&steering_ekf, Actuators_GetLastSteeringEffort(), 0.01f);
            // 2. Correct with fresh ADC measurement
            EKF_Update(&steering_ekf, raw_steering_v);

            float filtered_position_v = steering_ekf.x[0];
            float filtered_velocity_v = steering_ekf.x[1];

            // 3. Convert target angle (degrees) to target voltage using calibration map & polarity
            float target_volts = steering_limits.center_v
                                 + ((jetson_target_angle / steering_limits.degrees_per_volt) * (float)steering_limits.polarity_sign);

            // Clamp target voltage strictly within calibrated mechanical limits
            if (target_volts > steering_limits.limit_high_v - 0.05f) {
                target_volts = steering_limits.limit_high_v - 0.05f;
            }
            if (target_volts < steering_limits.limit_low_v + 0.05f) {
                target_volts = steering_limits.limit_low_v + 0.05f;
            }

            // 4. Run closed-loop PID (D-term uses EKF velocity, not finite-difference)
            float effort = PID_Calculate(&steering_pid,
                                         target_volts,
                                         filtered_position_v,
                                         filtered_velocity_v,
                                         0.01f);

            // Automatically correct for motor drive polarity!
            if (steering_limits.polarity_sign < 0) {
                effort = -effort;
            }

            // End-stop protection: prevent driving into mechanical stops or drawing stall current
            if (raw_steering_v <= (steering_limits.limit_low_v + 0.04f)) {
                if (steering_limits.polarity_sign > 0 && effort < 0.0f) effort = 0.0f;
                if (steering_limits.polarity_sign < 0 && effort > 0.0f) effort = 0.0f;
            }
            if (raw_steering_v >= (steering_limits.limit_high_v - 0.04f)) {
                if (steering_limits.polarity_sign > 0 && effort > 0.0f) effort = 0.0f;
                if (steering_limits.polarity_sign < 0 && effort < 0.0f) effort = 0.0f;
            }

            // Safety failsafe: if UART link is offline (not streaming from GUI), relax motor!
            if (!link_ok) {
                effort = 0.0f;
            }

            // 5. Drive steering motor
            Actuators_SetSteeringEffort(effort);
            break;
        }

        case STATE_FAULT:
        default:
            current_throttle_v = 0.0f;
            current_brake_v = 3.3f;
            Actuators_ApplySafeState();
            if (LedIndicator_GetError() == ERR_CODE_NONE) {
                LedIndicator_SetError(ERR_CODE_FATAL);
            }
            break;
    }

    // --- Update Unified Live Monitor Struct (100 Hz for CubeIDE Live Expressions) ---
    App_UpdateLiveMonitor(cmd, link_ok, raw_steering_v);

    // --- Update LED Visual Indicators (100 Hz non-blocking) ---
    LedIndicator_Update_100Hz((uint8_t)current_state, link_ok);

    // --- Telemetry Transmission (50 Hz / every 2 ticks) ---
    static uint8_t tlm_divider = 0;
    if (++tlm_divider >= 2) {
        tlm_divider = 0;
        // Broadcast telemetry when link is active OR when manually enabled via debugger
        if (Comm_IsTelemetryEnabled() || link_ok) {
            TelemetryPayload_t tlm;
            tlm.system_state = (uint8_t)current_state;
            tlm.actual_steer_angle = App_GetSteeringAngle();
            tlm.steering_pot_v = raw_steering_v;
            tlm.applied_steer_effort = Actuators_GetLastSteeringEffort();
            tlm.applied_throttle_v = current_throttle_v;
            tlm.applied_brake_v = current_brake_v;
            tlm.link_alive = link_ok ? 1 : 0;
            tlm.uptime_ms = HAL_GetTick();

            Comm_SendTelemetry(&tlm);
        }
    }
}
