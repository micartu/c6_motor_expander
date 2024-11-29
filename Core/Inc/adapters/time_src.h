#ifndef __TIME_SRC_H__
#define __TIME_SRC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"

struct val_time_t {
    uint32_t ms;
    uint32_t rest_us;
};

void init_system_timer(TIM_TypeDef * timer);
void sys_timer_irq(TIM_TypeDef * timer);
struct val_time_t get_current_time();

#ifdef __cplusplus
}
#endif

#endif /* __TIME_SRC_H__ */