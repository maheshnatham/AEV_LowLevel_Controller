#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define MSG_ID_CMD_CONTROL 0x01
#define MSG_ID_TLM_STATUS  0x81
#define COMM_TIMEOUT_MS    200

// Packed structure to match Jetson binary format exactly
typedef struct __attribute__((packed)) {
    uint8_t  drive_direction; // 0 = NEUTRAL, 1 = FORWARD, 2 = REVERSE
    int8_t   steer_direction; // -1 = LEFT, 0 = CENTER, +1 = RIGHT
    float    steer_angle;     // Steering angle in degrees
    float    acceleration;    // Throttle percentage: 0.0 to 100.0%
    uint8_t  brake;           // 0 = OFF, 1 = ON, 2 = EMERGENCY
    uint8_t  estop;           // 0 = NORMAL, 1 = EMERGENCY STOP
    uint32_t timestamp_ms;    // Millisecond timestamp from Jetson
} ControlPayload_t;

// Packed telemetry structure sent from STM32 back to Jetson / HTerm
typedef struct __attribute__((packed)) {
    uint8_t  system_state;         // 0=INIT, 1=CALIBRATING, 2=ACTIVE, 3=FAULT
    float    actual_steer_angle;   // Estimated angle in degrees
    float    steering_pot_v;       // Measured potentiometer voltage
    float    applied_steer_effort; // Motor PWM effort (-1.0 to +1.0)
    float    applied_throttle_v;   // DAC throttle voltage (0.0 to 3.3V)
    float    applied_brake_v;      // DAC brake voltage (0.0 to 3.3V)
    uint8_t  link_alive;           // 1 = Link active within 200ms, 0 = Timeout
    uint32_t uptime_ms;            // STM32 uptime in milliseconds
} TelemetryPayload_t;

void Comm_Init(void);
void Comm_Update_100Hz(void);
bool Comm_IsLinkActive(void);
uint32_t Comm_GetWatchdogRemainingMs(void);
ControlPayload_t Comm_GetLatestCommand(void);

// Telemetry & Test utilities
bool Comm_SendTelemetry(const TelemetryPayload_t *telemetry);
void Comm_SendString(const char *str);
uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length);
void Comm_EnableTelemetry(bool enable);
bool Comm_IsTelemetryEnabled(void);

// Live diagnostic variables (viewable in STM32CubeIDE Live Expressions)
extern volatile ControlPayload_t latest_command;
extern volatile uint32_t comm_rx_bytes;
extern volatile uint32_t comm_rx_packets;
extern volatile uint32_t comm_rx_crc_errs;

#endif /* PROTOCOL_H */
