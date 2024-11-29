#include "irq.h"

void wrp_disable_irq()
{
    __disable_irq();
}

void wrp_enable_irq()
{
    __enable_irq();
}