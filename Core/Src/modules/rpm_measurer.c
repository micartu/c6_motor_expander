#include "rpm_measurer.h"
#include "irq.h"
#include "time_src.h"
#include <string.h>

#define TIME_BASE_MS 1000000UL

void rpm_measurer_init(struct speed_measure_t * s)
{
    memset(s, 0, sizeof(*s));
}

void position_increased(struct speed_measure_t * rpm)
{
  ++(rpm->pos);
}

void position_decreased(struct speed_measure_t * rpm)
{

}

void calculate_rpm(struct speed_measure_t * rpm)
{
  wrp_disable_irq();
  // rpm->speed = rpm->pos * rpm->degrees_at_time;
  rpm->speed = rpm->pos;
  rpm->pos = 0; // reset current position
  wrp_enable_irq();
}

float current_speed(struct speed_measure_t * rpm)
{
  return rpm->speed;
}
