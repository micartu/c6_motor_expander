#include "time_src.h"
#include <stddef.h>

static TIM_TypeDef * _sys_timer = NULL;
static uint32_t _time = 0;

void init_system_timer(TIM_TypeDef * timer)
{
    _sys_timer = timer;
}

void sys_timer_irq(TIM_TypeDef * timer)
{
    if (timer == _sys_timer)
    {
        ++_time;
    }
}

void _set_sys_time(uint32_t time)
{
    _time = time;
}

struct val_time_t get_current_time()
{
    struct val_time_t val =
    {
        .ms = _time,
        .rest_us = 0
    };
    return val;
}