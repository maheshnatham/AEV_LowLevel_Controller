#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max, float i_limit) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_min = out_min; pid->output_max = out_max;
}

float PID_Calculate(PID_t *pid, float setpoint, float measured_pos, float measured_vel, float dt) {
    float error = setpoint - measured_pos;

    // Proportional & Derivative
    float p_term = pid->kp * error;
    float d_term = -(pid->kd * measured_vel); // Velocity is already derived by EKF

    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_term = pid->ki * pid->integral;

    float output = p_term + i_term + d_term;

    // Output clamping
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}
