#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdint.h>

void Actuators_Init(void);
void Actuators_SetThrottle_Volts(float volts);
void Actuators_SetBrake_Volts(float volts);
void Actuators_SetSteeringEffort(float effort);
float Actuators_GetLastSteeringEffort(void);
void Actuators_ApplySafeState(void);

// Telemetry / Live Expression diagnostic getters
uint32_t Actuators_GetThrottleDacCounts(void);
uint32_t Actuators_GetBrakeDacCounts(void);
uint8_t  Actuators_GetSteerDirPin(void);
float    Actuators_GetSteerPwmDutyPct(void);

#endif /* ACTUATORS_H */
