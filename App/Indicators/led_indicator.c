#include "led_indicator.h"
#include "main.h"

// Functional pin aliases matching CubeMX User Labels:
// PB0  -> SYS_STATUS (LD1 Green : System Operating State & Heartbeat)
// PE1  -> COMM_LINK  (LD2 Yellow: Autonomy Serial Link / Watchdog)
// PB14 -> SYS_FAULT  (LD3 Red   : Diagnostic Error Blink Codes)

#ifndef SYS_STATUS_Pin
#define SYS_STATUS_Pin       GPIO_PIN_0
#define SYS_STATUS_GPIO_Port GPIOB
#endif

#ifndef COMM_LINK_Pin
#define COMM_LINK_Pin       GPIO_PIN_1
#define COMM_LINK_GPIO_Port GPIOE
#endif

#ifndef SYS_FAULT_Pin
#define SYS_FAULT_Pin       GPIO_PIN_14
#define SYS_FAULT_GPIO_Port GPIOB
#endif

static ErrorCode_t active_error = ERR_CODE_NONE;
static uint16_t status_timer_ticks = 0;
static uint16_t fault_timer_ticks = 0;

void LedIndicator_Init(void) {
    // Ensure GPIO clocks for Port B and Port E are enabled
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // 1. SYS_STATUS (LD1 Green - PB0): System State & Heartbeat
    GPIO_InitStruct.Pin = SYS_STATUS_Pin;
    HAL_GPIO_Init(SYS_STATUS_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin, GPIO_PIN_RESET);

    // 2. COMM_LINK (LD2 Yellow - PE1): Autonomy Serial Link Status
    GPIO_InitStruct.Pin = COMM_LINK_Pin;
    HAL_GPIO_Init(COMM_LINK_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(COMM_LINK_GPIO_Port, COMM_LINK_Pin, GPIO_PIN_RESET);

    // 3. SYS_FAULT (LD3 Red - PB14): Diagnostic Fault Blink Codes
    GPIO_InitStruct.Pin = SYS_FAULT_Pin;
    HAL_GPIO_Init(SYS_FAULT_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_RESET);
}

void LedIndicator_SetError(ErrorCode_t error) {
    if (active_error != error) {
        active_error = error;
        fault_timer_ticks = 0; // Restart blink sequence immediately
    }
}

void LedIndicator_ClearError(void) {
    active_error = ERR_CODE_NONE;
    fault_timer_ticks = 0;
    HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_RESET);
}

ErrorCode_t LedIndicator_GetError(void) {
    return active_error;
}

void LedIndicator_Update_100Hz(uint8_t system_state, bool comm_active) {
    // -------------------------------------------------------------------------
    // 1. SYS_STATUS (LD1 Green - PB0) -> System State Engine
    // -------------------------------------------------------------------------
    status_timer_ticks++;

    if (system_state == 1) { // STATE_CALIBRATING: Rapid 5 Hz strobe (100ms ON / 100ms OFF)
        if (status_timer_ticks >= 20) status_timer_ticks = 0;
        HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin,
                          (status_timer_ticks < 10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else if (system_state == 2) { // STATE_ACTIVE: Gentle 1 Hz heartbeat (500ms ON / 500ms OFF)
        if (status_timer_ticks >= 100) status_timer_ticks = 0;
        HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin,
                          (status_timer_ticks < 50) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else { // STATE_FAULT or STATE_INIT: Status LED is OFF
        HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin, GPIO_PIN_RESET);
        status_timer_ticks = 0;
    }

    // -------------------------------------------------------------------------
    // 2. COMM_LINK (LD2 Yellow - PE1) -> Autonomy Link Watchdog Activity Blinker
    // -------------------------------------------------------------------------
    // Blinks rapidly (10 Hz: 50ms ON / 50ms OFF) while serial communication is active.
    // When a single test packet arrives, blinks for 1.0 second so each packet is clearly seen.
    // Turns completely OFF when communication link is idle or timed out (>200ms).
    static uint16_t comm_stretch_ticks = 0;
    static uint8_t comm_blink_ticks = 0;

    if (comm_active) {
        comm_stretch_ticks = 100; // Hold active window for 1.0s (100 ticks @ 100Hz)
    } else if (comm_stretch_ticks > 0) {
        comm_stretch_ticks--;
    }

    if (comm_stretch_ticks > 0) {
        comm_blink_ticks++;
        if (comm_blink_ticks >= 10) {
            comm_blink_ticks = 0;
        }
        HAL_GPIO_WritePin(COMM_LINK_GPIO_Port, COMM_LINK_Pin,
                          (comm_blink_ticks < 5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        comm_blink_ticks = 0;
        HAL_GPIO_WritePin(COMM_LINK_GPIO_Port, COMM_LINK_Pin, GPIO_PIN_RESET);
    }

    // -------------------------------------------------------------------------
    // 3. SYS_FAULT (LD3 Red - PB14) -> Diagnostic Error Blink Code Generator
    // -------------------------------------------------------------------------
    if (active_error == ERR_CODE_NONE) {
        HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_RESET);
        fault_timer_ticks = 0;
    }
    else if (active_error >= ERR_CODE_FATAL) {
        // Solid continuous ON for fatal/hardware error
        HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_SET);
    }
    else {
        // N blinks: Each blink = 200ms ON (20 ticks) + 200ms OFF (20 ticks) = 400ms (40 ticks)
        // Followed by 1200ms silent pause (120 ticks)
        uint8_t n_blinks = (uint8_t)active_error;
        uint16_t active_burst_ticks = n_blinks * 40;
        uint16_t total_period_ticks = active_burst_ticks + 120; // 1.2s pause

        if (fault_timer_ticks < active_burst_ticks) {
            uint16_t sub_tick = fault_timer_ticks % 40;
            HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin,
                              (sub_tick < 20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            // In the 1.2s pause window
            HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_RESET);
        }

        if (++fault_timer_ticks >= total_period_ticks) {
            fault_timer_ticks = 0;
        }
    }
}

void LedIndicator_TestSequence(void) {
    // Sequential sweep: SYS_STATUS -> COMM_LINK -> SYS_FAULT
    HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(SYS_STATUS_GPIO_Port, SYS_STATUS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(COMM_LINK_GPIO_Port, COMM_LINK_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(COMM_LINK_GPIO_Port, COMM_LINK_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(SYS_FAULT_GPIO_Port, SYS_FAULT_Pin, GPIO_PIN_RESET);
}
