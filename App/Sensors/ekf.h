#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x[3];       // State: [0]=Position, [1]=Velocity, [2]=Gain
    float P[3][3];    // Covariance matrix
    float R;          // Measurement noise variance
    float gainNom;    // Nominal gain
    uint8_t rejects;  // Outlier rejection counter
} EKF_t;

void EKF_Init(EKF_t *ekf, float p0, float g0, float r);
void EKF_Predict(EKF_t *ekf, float u, float dt);
bool EKF_Update(EKF_t *ekf, float z);
void EKF_ResetTo(EKF_t *ekf, float z);

#endif /* EKF_H */
