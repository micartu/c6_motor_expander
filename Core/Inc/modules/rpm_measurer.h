#ifndef __RPM_MEASURER_H__
#define __RPM_MEASURER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "time_src.h"

struct speed_measure_t
{
	/// current position of the wheel
	uint32_t pos;
	/// measured speed at time <last_measured_time>
	float speed;
};

/**
 * Initializes measurer unit struct
 * which contains all information needed to calculate the RPM of a wheel
 *
 * @param s structure to be initialized
 */
void rpm_measurer_init(struct speed_measure_t * s);

/**
 * Calculates RPM based on previous values
 * Must be called in IRQ from EXT Output's PIN
 *
 * @param rpm structure which contains all needed data for measuring RPM
 */
void calculate_rpm(struct speed_measure_t * rpm);

/**
 * @brief returns the current estimated speed of the wheel
 * 
 * @param rpm all data needed to estimate the spee
 * @return currenly estimated speed
 */
float current_speed(struct speed_measure_t * rpm);

/**
 * @brief increases the current position of the wheel
 * 
 * @param rpm all data needed to estimate the spee
 */
void position_increased(struct speed_measure_t * rpm);

/**
 * @brief decreases the current position of the wheel
 * 
 * @param rpm all data needed to estimate the spee
 */
void position_decreased(struct speed_measure_t * rpm);

#ifdef __cplusplus
}
#endif

#endif /* __RPM_MEASURER_H__ */
