#include "ekf.h"
#include <math.h>

#define EKF_TAU          0.08f
#define EKF_Q_POS        2.0f
#define EKF_Q_VEL        4.0e4f
#define EKF_Q_GAIN       0.02f
#define EKF_IDLE_Q_SCALE 0.05f
#define EKF_GATE         6.0f

static inline float constrain_f(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void EKF_Init(EKF_t *ekf, float p0, float g0, float r) {
    ekf->x[0] = p0;
    ekf->x[1] = 0.0f;
    ekf->x[2] = g0;
    ekf->gainNom = g0;
    ekf->R = r;
    ekf->rejects = 0;

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) ekf->P[i][j] = 0.0f;
    }
    ekf->P[0][0] = r;
    ekf->P[1][1] = 400.0f;
    ekf->P[2][2] = (0.3f * g0) * (0.3f * g0);
}

void EKF_Predict(EKF_t *ekf, float u, float dt) {
    float a = dt / EKF_TAU;
    if (a > 1.0f) a = 1.0f;

    float v = ekf->x[1];
    float g = ekf->x[2];

    ekf->x[0] += v * dt;
    ekf->x[1] = v + a * (g * u - v);

    float F[3][3] = {
        {1.0f, dt, 0.0f},
        {0.0f, 1.0f - a, a * u},
        {0.0f, 0.0f, 1.0f}
    };

    float FP[3][3];
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            FP[i][j] = F[i][0] * ekf->P[0][j] + F[i][1] * ekf->P[1][j] + F[i][2] * ekf->P[2][j];
        }
    }

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            ekf->P[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
        }
    }

    ekf->P[0][0] += EKF_Q_POS * dt;
    ekf->P[1][1] += EKF_Q_VEL * dt * (fabsf(u) > 1e-3f ? 1.0f : EKF_IDLE_Q_SCALE);
    ekf->P[2][2] += (EKF_Q_GAIN * ekf->gainNom) * (EKF_Q_GAIN * ekf->gainNom) * dt;
}

bool EKF_Update(EKF_t *ekf, float z) {
    float y = z - ekf->x[0];
    float S = ekf->P[0][0] + ekf->R;

    // Glitch rejection gate
    if (y * y > EKF_GATE * EKF_GATE * S) {
        if (ekf->rejects < 255) ekf->rejects++;
        return false;
    }

    ekf->rejects = 0;
    float K[3] = { ekf->P[0][0] / S, ekf->P[1][0] / S, ekf->P[2][0] / S };
    float P0[3] = { ekf->P[0][0], ekf->P[0][1], ekf->P[0][2] };

    for (uint8_t i = 0; i < 3; i++) {
        ekf->x[i] += K[i] * y;
        for (uint8_t j = 0; j < 3; j++) ekf->P[i][j] -= K[i] * P0[j];
    }

    // Enforce symmetry
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = i + 1; j < 3; j++) {
            float m = 0.5f * (ekf->P[i][j] + ekf->P[j][i]);
            ekf->P[i][j] = ekf->P[j][i] = m;
        }
    }

    ekf->x[2] = constrain_f(ekf->x[2], 0.3f * ekf->gainNom, 3.0f * ekf->gainNom);
    return true;
}

void EKF_ResetTo(EKF_t *ekf, float z) {
    ekf->x[0] = z;
    ekf->x[1] = 0.0f;
    ekf->P[0][0] = ekf->R;
    ekf->P[1][1] = 400.0f;
    ekf->P[0][1] = ekf->P[1][0] = 0.0f;
    ekf->rejects = 0;
}
