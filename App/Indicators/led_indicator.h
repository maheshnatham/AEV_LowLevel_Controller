#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Diagnostic Fault Codes (Blink codes on SYS_FAULT / Red LED LD3 - PB14)
 */
typedef enum {
    ERR_CODE_NONE                 = 0, // No fault (SYS_FAULT LED is OFF)
    ERR_CODE_COMM_TIMEOUT         = 1, // 1 Blink  - Watchdog timeout (>200ms)
    ERR_CODE_CALIBRATION_FAIL     = 2, // 2 Blinks - Endstop stall / sweep timeout
    ERR_CODE_SENSOR_OUT_OF_BOUNDS = 3, // 3 Blinks - Potentiometer disconnected (<0.15V or >3.15V)
    ERR_CODE_ESTOP_ACTIVE         = 4, // 4 Blinks - Emergency Stop asserted
    ERR_CODE_FATAL                = 5  // Solid ON - Hardware/critical fault
} ErrorCode_t;

/**
 * @brief Initializes GPIO clocks and pins for:
 *        - SYS_STATUS (LD1 Green  - PB0) : System Operating Mode & Heartbeat
 *        - COMM_LINK  (LD2 Yellow - PE1) : Autonomy Serial Link Status
 *        - SYS_FAULT  (LD3 Red    - PB14): Diagnostic Error Blink Codes
 */
void LedIndicator_Init(void);

/**
 * @brief Periodic 100 Hz state machine updating all 3 LEDs (called from App_Step_100Hz).
 * @param system_state Current state (0=INIT, 1=CALIBRATING, 2=ACTIVE, 3=FAULT)
 * @param comm_active true if Jetson communication watchdog is healthy (<=200ms)
 */
void LedIndicator_Update_100Hz(uint8_t system_state, bool comm_active);

/**
 * @brief Sets an active diagnostic error code for the Red LED blink engine.
 */
void LedIndicator_SetError(ErrorCode_t error);

/**
 * @brief Clears the active error code and turns off the Red LED.
 */
void LedIndicator_ClearError(void);

/**
 * @brief Returns the current active error code.
 */
ErrorCode_t LedIndicator_GetError(void);

/**
 * @brief Quick visual self-test: sweeps Green -> Yellow -> Red LEDs.
 */
void LedIndicator_TestSequence(void);

#endif /* LED_INDICATOR_H */
