#include "sensors.h"
#include "adc.h"

#define ADC_MAX_COUNTS 65535.0f // 16-bit resolution
#define ADC_VREF       3.3f
#define EMA_ALPHA      0.2f     // Low-pass filter coefficient for the EKF input

// The DMA controller will continuously write to this array in the background
uint16_t adc_dma_buf[ADC_DMA_BUF_SIZE];

static float filtered_adc_avg = 32768.0f; // Start at assumed mid-point
static float steering_volts = 1.65f;

void Sensors_Init(void) {
    // 1. Explicitly calibrate the STM32H7 ADC before use to prevent static offsets
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // 2. Start the ADC in Circular DMA Mode
    // The hardware will now continuously fill adc_dma_buf without CPU intervention
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_DMA_BUF_SIZE);
}

void Sensors_Update_100Hz(void) {
    uint32_t sum = 0;

    // 1. Instantly sum the 64 samples collected by the DMA over the last 10ms
    for (uint16_t i = 0; i < ADC_DMA_BUF_SIZE; i++) {
        sum += adc_dma_buf[i];
    }

    float current_block_avg = (float)sum / ADC_DMA_BUF_SIZE;

    // 2. Apply Exponential Moving Average (EMA) to smooth out any residual inverter EMI
    filtered_adc_avg = (EMA_ALPHA * current_block_avg) + ((1.0f - EMA_ALPHA) * filtered_adc_avg);

    // 3. Convert raw counts to physical voltage
    steering_volts = (filtered_adc_avg / ADC_MAX_COUNTS) * ADC_VREF;
}

float Sensors_GetSteeringRawAvg(void) {
    return filtered_adc_avg;
}

float Sensors_GetSteeringVolts(void) {
    return steering_volts;
}
