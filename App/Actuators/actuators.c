#include "actuators.h"
#include "dac.h"
#include "tim.h"  // For PWM
#include "gpio.h" // For DIR pin

#define V_REF           3.3f
#define DAC_MAX_COUNTS  4095.0f

// Replace this with your actual Counter Period (ARR) from CubeMX
#define PWM_MAX_PERIOD  13750.0f

static float last_steering_effort = 0.0f;

static uint32_t VoltsToDac(float volts) {
    if (volts < 0.0f) volts = 0.0f;
    if (volts > V_REF) volts = V_REF;
    return (uint32_t)((volts / V_REF) * DAC_MAX_COUNTS);
}

void Actuators_Init(void) {
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    // Start the steering PWM timer
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    Actuators_ApplySafeState();
}

void Actuators_SetThrottle_Volts(float volts) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VoltsToDac(volts));
}

void Actuators_SetBrake_Volts(float volts) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, VoltsToDac(volts));
}

void Actuators_SetSteeringEffort(float effort) {
    // Clamp effort between -1.0 (Full Left) and 1.0 (Full Right)
    if (effort > 1.0f) effort = 1.0f;
    if (effort < -1.0f) effort = -1.0f;

    last_steering_effort = effort;

    // Handle Direction (Cytron logic: High/Low dictates Left/Right)
    if (effort >= 0.0f) {
        HAL_GPIO_WritePin(STEER_DIR_GPIO_Port, STEER_DIR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(STEER_DIR_GPIO_Port, STEER_DIR_Pin, GPIO_PIN_RESET);
        effort = -effort; // Convert to positive magnitude for PWM
    }

    // Write Duty Cycle to Timer Compare Register
    uint32_t pwm_counts = (uint32_t)(effort * PWM_MAX_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_counts);
}

float Actuators_GetLastSteeringEffort(void) {
    return last_steering_effort;
}

void Actuators_ApplySafeState(void) {
    Actuators_SetThrottle_Volts(0.0f);
    Actuators_SetBrake_Volts(0.0f);
    Actuators_SetSteeringEffort(0.0f);
}

uint32_t Actuators_GetThrottleDacCounts(void) {
    return HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1);
}

uint32_t Actuators_GetBrakeDacCounts(void) {
    return HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_2);
}

uint8_t Actuators_GetSteerDirPin(void) {
    return (HAL_GPIO_ReadPin(STEER_DIR_GPIO_Port, STEER_DIR_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

float Actuators_GetSteerPwmDutyPct(void) {
    return ((float)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) / PWM_MAX_PERIOD) * 100.0f;
}

