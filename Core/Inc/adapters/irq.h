#ifndef __IRQ_H__
#define __IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void wrp_disable_irq();
void wrp_enable_irq();

#ifdef __cplusplus
}
#endif

#endif /* __IRQ_H__ */