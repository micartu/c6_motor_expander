#include "pid.h"
#include <string.h>
#include <float.h>

void init_pid(struct cntr_pid_t * pid, float kp, float ki, float kd, float tau, float fmin, float fmax)
{
    memset(pid, 0, sizeof(*pid));

    pid->tau = tau;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->lim_min = fmin;
    pid->lim_max = fmax;

    pid->lim_max_int = FLT_MAX;
    pid->lim_min_int = FLT_MIN;
}

void reset_pid(struct cntr_pid_t * pid)
{
    pid->out = 0;
    pid->integrator = 0;
    pid->differentiator = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
}

void set_integrator_limits(struct cntr_pid_t * pid, float fmin, float fmax)
{
    pid->lim_min_int = fmin;
    pid->lim_max_int = fmax;
}

void set_sample_time(struct cntr_pid_t * pid, float ts)
{
    pid->ts = ts;
}

float pid_update(struct cntr_pid_t * pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    float proportional = pid->kp * error;
    pid->integrator = pid->integrator + 0.5f * pid->ki * pid->ts * (error + pid->prev_error);

    if (pid->integrator > pid->lim_max_int) {
        pid->integrator = pid->lim_max_int;
    } else if (pid->integrator < pid->lim_min_int) {
        pid->integrator = pid->lim_min_int;
    }

	// Derivative (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->kd * (measurement - pid->prev_measurement)	// Note: derivative on measurement, therefore minus sign in front of equation! 
                        + (2.0f * pid->tau - pid->ts) * pid->differentiator)
                        / (2.0f * pid->tau + pid->ts);

	// Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->lim_max) {
        pid->out = pid->lim_max;
    } else if (pid->out < pid->lim_min) {
        pid->out = pid->lim_min;
    }

    // Store error and measurement for later use
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    // Return controller output
    return pid->out;
}