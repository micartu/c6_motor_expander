#include "drv_starter.h"
#include "consts.h"
#include <math.h>

int is_starter_applied(struct motor_action_t * mtr,
                       enum motor_side_t side,
                       enum motor_state_t where,
                       float speed,
                       float sp)
{
    if (fabs(speed) < EPS_SPEED && (where == FORWARD || where == BACKWARD))
    {
        // try to apply maximum torque to motor
        // in order to force its rotation
        apply_action_to_motor(mtr, side, where, 64000);
        return STARTER_MAX_TORQUE;
    }
    return 0;
}