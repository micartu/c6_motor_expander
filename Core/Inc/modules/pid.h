#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

struct cntr_pid_t 
{
	// Controller gains
    /// Kp proportional gain
	float kp;
    /// Ki integral koeficient
	float ki;
    /// Kd differential koeficient
	float kd;

	/// Derivative low-pass filter time constant
	float tau;

	// Output limits

    /// minimum for output
	float lim_min;

    /// maximum for output
	float lim_max;
	
	// Integrator limits
	float lim_min_int;
	float lim_max_int;

	/// Sample time (in seconds)
	float ts;

	/// Controller "memory"
	float integrator;
	float prev_error;			/// required for integrator
	float differentiator;
	float prev_measurement;		/// Required for differentiator

	/// Controller output
	float out;
};

void init_pid(struct cntr_pid_t * pid, float kp, float ki, float kd, float tau, float fmin, float fmax);
void reset_pid(struct cntr_pid_t * pid);
void set_integrator_limits(struct cntr_pid_t * pid, float fmin, float fmax);
void set_sample_time(struct cntr_pid_t * pid, float ts);
float pid_update(struct cntr_pid_t * pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */