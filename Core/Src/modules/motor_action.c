#include "motor_action.h"
#include <string.h>

static void init_motor_control(struct motor_control_t * motor, __IO uint32_t * CCRM0, __IO uint32_t * CCRM1)
{
    motor->CCR0 = CCRM0;
    motor->CCR1 = CCRM1;
}

void init_motor_action(struct motor_action_t * mtr, __IO uint32_t * CCRM0, __IO uint32_t * CCRM1, __IO uint32_t * CCRM2, __IO uint32_t * CCRM3)
{
    memset(mtr, 0, sizeof(*mtr));
    init_motor_control(&mtr->m0, CCRM0, CCRM1);
    init_motor_control(&mtr->m1, CCRM2, CCRM3);
}

static inline struct motor_control_t * get_motor(struct motor_action_t * mtr, enum motor_side_t side)
{
    if (side == MRIGHT)
        return &mtr->m0;
    return &mtr->m1;
}

float speed_sign_of(struct motor_action_t * mtr, enum motor_side_t side)
{
    struct motor_control_t * m = get_motor(mtr, side);
    if (*(m->CCR0) > 0 && *(m->CCR1) == 0)
        return 1;
    else if (*(m->CCR0) == 0 && *(m->CCR1) > 0)
        return -1;
    return 0;
}

void apply_action_to_motor(struct motor_action_t * mtr, enum motor_side_t side, enum motor_state_t action, uint16_t adc)
{
    struct motor_control_t * m = get_motor(mtr, side);
    switch (action)
    {
    case FORWARD:
        *(m->CCR0) = adc;
        *(m->CCR1) = 0;
        break;

    case BACKWARD:
        *(m->CCR0) = 0;
        *(m->CCR1) = adc;
        break;

    case BRAKE:
        *(m->CCR0) = MAX_TORQUE;
        *(m->CCR1) = MAX_TORQUE;
        break;
    
    default: // IDLE
        *(m->CCR0) = 0;
        *(m->CCR1) = 0;
        break;
    }
}
