#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

// Calibration Configuration
#define CAL_SWEEP_EFFORT       0.45f   // 45% motor effort during sweep (authoritative torque across rack)
#define CAL_MIN_SPAN_V         0.25f   // Minimum valid voltage span between physical end-stops (250mV)
#define CAL_STALL_CONFIRM_MS   500     // 500ms stationary confirmation to declare physical end-stop
#define CAL_STARTUP_GRACE_MS   600     // 600ms grace period allowing motor to overcome static inertia
#define CAL_SWEEP_TIMEOUT_MS   10000   // 10.0s timeout per sweep direction to prevent runaway

typedef enum {
    CAL_STATE_INIT,
    CAL_STATE_SWEEP_DIR_1,
    CAL_STATE_BACKOFF,
    CAL_STATE_SWEEP_DIR_2,
    CAL_STATE_CENTERING,
    CAL_STATE_DONE,
    CAL_STATE_FAULT
} CalState_t;

typedef struct {
    float  limit_high_v;
    float  limit_low_v;
    float  center_v;
    float  degrees_per_volt;
    int8_t polarity_sign; // +1 if DIR=SET increases volts, -1 if DIR=SET decreases volts
} CalData_t;

void Calibration_Init(void);
CalState_t Calibration_Step_100Hz(float current_volts, float *out_effort);
CalData_t Calibration_GetResults(void);

#endif /* CALIBRATION_H */
