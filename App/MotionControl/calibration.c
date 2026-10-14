#include "calibration.h"
#include <math.h>

// Physical steering properties (Adjust based on your vehicle)
#define TOTAL_STEERING_ANGLE_DEG 60.0f // e.g., -30 to +30 degrees

static CalState_t state = CAL_STATE_INIT;
static CalData_t results = {0};

static uint32_t state_time_ms = 0;       // Total time spent in the current state
static uint32_t stall_timer_ms = 0;      // Milliseconds the motor has been stationary
static float sample_volts = 0.0f;        // Voltage sampled at start of movement window
static uint32_t sample_window_ms = 0;    // 100ms window timer for movement evaluation

static float limit_1 = 0.0f;
static float limit_2 = 0.0f;

void Calibration_Init(void) {
    state = CAL_STATE_INIT;
    state_time_ms = 0;
    stall_timer_ms = 0;
    sample_window_ms = 0;
    sample_volts = 0.0f;
    limit_1 = 0.0f;
    limit_2 = 0.0f;
}

CalState_t Calibration_Step_100Hz(float current_volts, float *out_effort) {
    state_time_ms += 10;     // 100Hz loop = 10ms per step
    sample_window_ms += 10;

    // Evaluate movement across a 100ms sliding window (immune to ADC noise)
    if (sample_window_ms >= 100) {
        float movement = fabsf(current_volts - sample_volts);
        sample_volts = current_volts;
        sample_window_ms = 0;

        // If moved more than 5mV in 100ms (~0.05 V/s), the motor is actively turning
        if (movement > 0.005f) {
            stall_timer_ms = 0; // Reset stall timer — motor is moving!
        } else {
            stall_timer_ms += 100; // Accumulate stall time ONLY when stationary across window
        }
    }

    switch (state) {
        case CAL_STATE_INIT:
            *out_effort = 0.0f;
            state_time_ms = 0;
            stall_timer_ms = 0;
            sample_window_ms = 0;
            sample_volts = current_volts;
            state = CAL_STATE_SWEEP_DIR_1;
            break;

        case CAL_STATE_SWEEP_DIR_1:
            *out_effort = CAL_SWEEP_EFFORT; // Drive positive direction

            // Only check stall after startup grace period (gives motor time to overcome static friction)
            if (state_time_ms >= CAL_STARTUP_GRACE_MS && stall_timer_ms >= CAL_STALL_CONFIRM_MS) {
                limit_1 = current_volts; // First physical end-stop reached!
                state_time_ms = 0;
                stall_timer_ms = 0;
                sample_window_ms = 0;
                sample_volts = current_volts;
                state = CAL_STATE_BACKOFF;
            } else if (state_time_ms >= CAL_SWEEP_TIMEOUT_MS) {
                state = CAL_STATE_FAULT; // Took longer than timeout
            }
            break;

        case CAL_STATE_BACKOFF:
            *out_effort = -CAL_SWEEP_EFFORT; // Reverse briefly to clear the end-stop
            if (state_time_ms >= 500) {      // Back off for 500ms
                state_time_ms = 0;
                stall_timer_ms = 0;
                sample_window_ms = 0;
                sample_volts = current_volts;
                state = CAL_STATE_SWEEP_DIR_2;
            }
            break;

        case CAL_STATE_SWEEP_DIR_2:
            *out_effort = -CAL_SWEEP_EFFORT; // Drive negative direction toward second end-stop

            if (state_time_ms >= CAL_STARTUP_GRACE_MS && stall_timer_ms >= CAL_STALL_CONFIRM_MS) {
                limit_2 = current_volts; // Second physical end-stop reached!

                // Sort high and low limits mathematically
                if (limit_1 > limit_2) {
                    results.limit_high_v = limit_1;
                    results.limit_low_v = limit_2;
                    results.polarity_sign = 1;  // +effort drove towards high voltage
                } else {
                    results.limit_high_v = limit_2;
                    results.limit_low_v = limit_1;
                    results.polarity_sign = -1; // +effort drove towards low voltage
                }

                // Calculate geometric center and voltage span
                results.center_v = (results.limit_high_v + results.limit_low_v) / 2.0f;
                float voltage_span = results.limit_high_v - results.limit_low_v;

                // Validate minimum physical span
                if (voltage_span < CAL_MIN_SPAN_V) {
                    state = CAL_STATE_FAULT; // Span too narrow
                    break;
                }

                results.degrees_per_volt = TOTAL_STEERING_ANGLE_DEG / voltage_span;

                state_time_ms = 0;
                state = CAL_STATE_CENTERING;
            } else if (state_time_ms >= CAL_SWEEP_TIMEOUT_MS) {
                state = CAL_STATE_FAULT;
            }
            break;

        case CAL_STATE_CENTERING:
        {
            // Self-orienting Proportional control to snap to center
            float error = results.center_v - current_volts;
            float dir_sign = (results.polarity_sign > 0) ? 1.0f : -1.0f;
            float effort = error * 2.5f * dir_sign;

            // Clamp effort to safe centering speed (max 35%)
            if (effort > 0.35f) effort = 0.35f;
            if (effort < -0.35f) effort = -0.35f;
            *out_effort = effort;

            // Once within 30mV of mechanical center (or after 2.5 seconds), calibration is complete!
            if (fabsf(error) < 0.030f || state_time_ms >= 2500) {
                *out_effort = 0.0f;
                state = CAL_STATE_DONE;
            }
            break;
        }

        case CAL_STATE_DONE:
            *out_effort = 0.0f;
            break;

        case CAL_STATE_FAULT:
        default:
            *out_effort = 0.0f;
            break;
    }

    return state;
}

CalData_t Calibration_GetResults(void) {
    return results;
}

