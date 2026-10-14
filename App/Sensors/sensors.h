#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define ADC_DMA_BUF_SIZE 64

void Sensors_Init(void);
void Sensors_Update_100Hz(void);

float Sensors_GetSteeringVolts(void);
float Sensors_GetSteeringRawAvg(void);

#endif /* SENSORS_H */
