#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float output_min, output_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max, float i_limit);
float PID_Calculate(PID_t *pid, float setpoint, float measured_pos, float measured_vel, float dt);

#endif /* PID_H */
