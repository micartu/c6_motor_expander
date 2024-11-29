#ifndef __DRV_STARTER_H__
#define __DRV_STARTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "motor_action.h"

#define STARTER_BREAK 444
#define STARTER_MAX_TORQUE 333


int is_starter_applied(struct motor_action_t * mtr,
                       enum motor_side_t side,
                       enum motor_state_t where,
                       float speed,
                       float sp);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_STARTER_H__ */