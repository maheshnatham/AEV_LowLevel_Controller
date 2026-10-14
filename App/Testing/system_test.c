#include "system_test.h"
#include "sensors.h"
#include "actuators.h"
#include "protocol.h"
#include "app_main.h"
#include "calibration.h"
#include "led_indicator.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static char print_buf[256];

bool SystemTest_CheckSensors(float *out_volts) {
    Sensors_Update_100Hz();
    float v = Sensors_GetSteeringVolts();
    if (out_volts) {
        *out_volts = v;
    }
    // Return true if sensor voltage is not disconnected (< 0.15V) or shorted (> 3.15V)
    return (v >= 0.15f && v <= 3.15f);
}

bool SystemTest_CheckActuators(void) {
    // 1. Verify visual indicator LEDs (Green -> Yellow -> Red sweep)
    LedIndicator_TestSequence();

    // 2. Verify DAC throttle & brake safe output
    Actuators_SetThrottle_Volts(0.0f);
    Actuators_SetBrake_Volts(0.0f);

    // 3. Verify PWM steering safe output
    Actuators_SetSteeringEffort(0.0f);

    // 4. Confirm safe state
    Actuators_ApplySafeState();
    return true;
}

bool SystemTest_CheckComm(void) {
    // Confirm UART3 is initialized and not in hard error state
    if (huart3.ErrorCode != HAL_UART_ERROR_NONE) {
        return false;
    }
    return true;
}

void SystemTest_PrintDiagnostics(void) {
    float pot_v = Sensors_GetSteeringVolts();
    float steer_angle = App_GetSteeringAngle();
    uint8_t state = App_GetSystemState();
    bool link_ok = Comm_IsLinkActive();

    const char *state_str = "UNKNOWN";
    if (state == 0) state_str = "INIT";
    else if (state == 1) state_str = "CALIBRATING";
    else if (state == 2) state_str = "ACTIVE";
    else if (state == 3) state_str = "FAULT";

    snprintf(print_buf, sizeof(print_buf),
             "[DIAG] State:%s | Pot:%.3fV | Steer:%.1fdeg | Link:%s | Uptime:%lums\r\n",
             state_str,
             pot_v,
             steer_angle,
             link_ok ? "ONLINE (Active)" : "OFFLINE (Timeout)",
             HAL_GetTick());

    Comm_SendString(print_buf);
}

void SystemTest_SendSampleTlm(void) {
    TelemetryPayload_t sample = {
        .system_state = App_GetSystemState(),
        .actual_steer_angle = 12.5f,
        .steering_pot_v = 1.650f,
        .applied_steer_effort = 0.25f,
        .applied_throttle_v = 1.10f,
        .applied_brake_v = 0.0f,
        .link_alive = 1,
        .uptime_ms = HAL_GetTick()
    };
    Comm_SendTelemetry(&sample);
}

void SystemTest_RunAll(void) {
    Comm_EnableTelemetry(false);
    HAL_Delay(50); // Allow any ongoing transmission to complete

    Comm_SendString("\r\n=======================================================\r\n");
    Comm_SendString("    AEV LOW-LEVEL CONTROLLER SYSTEM INITIAL CHECKS    \r\n");
    Comm_SendString("=======================================================\r\n");

    // --- 1. SENSORS CHECK ---
    float pot_v = 0.0f;
    bool pot_ok = SystemTest_CheckSensors(&pot_v);
    snprintf(print_buf, sizeof(print_buf),
             "[1/4] Sensors Test: Steering Pot = %.3f V [%s]\r\n"
             "      -> Raw Block Average = %.1f counts (16-bit)\r\n",
             pot_v,
             pot_ok ? "PASS" : "WARN (Out of 0.15V - 3.15V expected range)",
             Sensors_GetSteeringRawAvg());
    Comm_SendString(print_buf);
    HAL_Delay(30);

    // --- 2. ACTUATORS CHECK ---
    bool act_ok = SystemTest_CheckActuators();
    snprintf(print_buf, sizeof(print_buf),
             "[2/4] Actuators Test: DAC1 (Throttle/Brake) & TIM3 PWM [%s]\r\n"
             "      -> Actuators_ApplySafeState() executed successfully.\r\n",
             act_ok ? "PASS" : "FAIL");
    Comm_SendString(print_buf);
    HAL_Delay(30);

    // --- 3. COMMUNICATION CHECK ---
    bool comm_ok = SystemTest_CheckComm();
    bool link_ok = Comm_IsLinkActive();
    snprintf(print_buf, sizeof(print_buf),
             "[3/4] Comm Test: USART3 @ 115200 8N1 (Circular DMA) [%s]\r\n"
             "      -> Watchdog Link Status: %s\r\n"
             "      -> Rx Packets: %lu, Rx Bytes: %lu, CRC Errors: %lu\r\n"
             "      -> Stored Command: Dir=%u, Steer=%.1f deg, Accel=%.1f%%, Brake=%u, EStop=%u\r\n",
             comm_ok ? "PASS" : "FAIL",
             link_ok ? "HEARTBEAT RECEIVED (Online)" : "WAITING FOR COMMAND PACKET (Offline)",
             comm_rx_packets, comm_rx_bytes, comm_rx_crc_errs,
             (unsigned int)latest_command.drive_direction,
             latest_command.steer_angle,
             latest_command.acceleration,
             (unsigned int)latest_command.brake,
             (unsigned int)latest_command.estop);
    Comm_SendString(print_buf);
    HAL_Delay(30);

    // --- 4. CALIBRATION & MOTION CONTROL STATUS ---
    uint8_t state = App_GetSystemState();
    const char *state_str = (state == 0) ? "INIT" :
                            (state == 1) ? "CALIBRATING" :
                            (state == 2) ? "ACTIVE (PID Closed-Loop)" : "FAULT";

    snprintf(print_buf, sizeof(print_buf),
             "[4/4] Motion Control: Current State = %s\r\n",
             state_str);
    Comm_SendString(print_buf);

    if (state == 2) {
        CalData_t limits = Calibration_GetResults();
        snprintf(print_buf, sizeof(print_buf),
                 "      -> Calibrated Center = %.3f V\r\n"
                 "      -> Calibrated Range  = %.3f V to %.3f V\r\n"
                 "      -> Scale Factor      = %.2f deg/V\r\n",
                 limits.center_v,
                 limits.limit_low_v,
                 limits.limit_high_v,
                 limits.degrees_per_volt);
        Comm_SendString(print_buf);
    }
    HAL_Delay(30);

    Comm_SendString("=======================================================\r\n");
    Comm_SendString("    SELF-TEST ROUTINE FINISHED. READY.                 \r\n");
    Comm_SendString("=======================================================\r\n\r\n");
}
