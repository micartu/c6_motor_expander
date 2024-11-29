#include "CppUTest/TestHarness.h"
#include "drv_starter.h"
#include <assert.h>

/// a shortcut for assiging timer's CCR register
#define CCR(tim, num) (&(tim)->CCR##num)

TEST_GROUP(DrvStarter)
{
    struct motor_action_t mtr;

    // fake timer descriptors
    TIM_TypeDef tim0;
    TIM_TypeDef tim1;

    void setup()
    {
        init_motor_action(&mtr,
                          CCR(&tim0, 1),
                          CCR(&tim0, 2),
                          CCR(&tim1, 1),
                          CCR(&tim1, 2));
    }

    void teardown()
    {
    }
};

TEST(DrvStarter, MoveForwardFromStoppedState)
{
    // given
    float speed = 0;

    // when
    long applied = is_starter_applied(&mtr, MLEFT, FORWARD, speed, 4);

    // then
    LONGS_EQUAL(STARTER_MAX_TORQUE, applied);
}

TEST(DrvStarter, DoNotApplyStarterIfAlreadyMoving)
{
    // given
    float speed = 10;

    // when
    long applied = is_starter_applied(&mtr, MLEFT, FORWARD, speed, 4);

    // then
    LONGS_EQUAL(0, applied);
}