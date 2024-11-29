#ifndef __MOTOR_ACTION_H__
#define __MOTOR_ACTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32f1xx_hal.h"

#define MAX_TORQUE		65535

struct motor_control_t
{
    __IO uint32_t * CCR0; //<! a pointer to first PWM control register
    __IO uint32_t * CCR1; //<! a pointer to second PWM control register
};


/**
  * describes state of two motor which attached
  * on the same axis
  */
struct motor_action_t
{
    struct motor_control_t m0;
    struct motor_control_t m1;
};

/**
  * describes state of the motor and in which
  * direction it rotates attached to it wheel
  */
enum motor_state_t
{
    IDLE_TORQUE,
    FORWARD,
    BACKWARD,
    BRAKE
};

/**
  * which motor out of two to use
  */
enum motor_side_t
{
    MLEFT,
    MRIGHT
};

/**
 * Initializes motor's controlling struct
 * 
 * @param motor structure to be initialized
 * @param CCRM array of CCR registers to be used for controlling motors
 */
void init_motor_action(struct motor_action_t * mtr, __IO uint32_t * CCRM0, __IO uint32_t * CCRM1, __IO uint32_t * CCRM2, __IO uint32_t * CCRM3);

/**
 * @brief return current direction (speed sign) where the wheel moves
 * 
 * @param mtr bundle with info about 2 motors
 * @param side what wheel should be addressed
 * @return direction sign or 0 if the wheel doesn't move
 */
float speed_sign_of(struct motor_action_t * mtr, enum motor_side_t side);

/**
 * Applies an action to selected motor 
 * 
 * @param motor structure to be initialized
 * @param side which motor must be used (left or right)
 * @param action what exactly the wheel should do
 * @param adc procentage of torque applied to a motor
 */
void apply_action_to_motor(struct motor_action_t * mtr, enum motor_side_t side, enum motor_state_t action, uint16_t adc);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_ACTION_H__ */