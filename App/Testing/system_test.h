#ifndef SYSTEM_TEST_H
#define SYSTEM_TEST_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Runs all system diagnostic checks and transmits a complete
 *        human-readable diagnostic report over UART (viewable in HTerm ASCII mode),
 *        followed by a sample binary telemetry packet (viewable in HTerm Hex mode).
 */
void SystemTest_RunAll(void);

/**
 * @brief Checks if the steering potentiometer voltage is within reasonable electrical bounds.
 * @param out_volts Pointer to float to receive the measured voltage.
 * @return true if voltage is within [0.15V, 3.15V], false if open/shorted.
 */
bool SystemTest_CheckSensors(float *out_volts);

/**
 * @brief Verifies that actuator interfaces (DAC1 CH1, DAC1 CH2, TIM3 PWM, DIR GPIO)
 *        are initialized, executes safe test writes, and reapplies safe state.
 * @return true if successful.
 */
bool SystemTest_CheckActuators(void);

/**
 * @brief Verifies UART3 and DMA reception health, reporting link activity.
 * @return true if UART and DMA are operational.
 */
bool SystemTest_CheckComm(void);

/**
 * @brief Prints real-time system metrics (state, voltage, angle, throttle, brake, watchdog)
 *        as a single-line ASCII status for continuous monitoring in HTerm.
 */
void SystemTest_PrintDiagnostics(void);

/**
 * @brief Sends a known sample binary telemetry packet with valid CRC-16
 *        to verify HTerm Hex reception decoding.
 */
void SystemTest_SendSampleTlm(void);

#endif /* SYSTEM_TEST_H */
