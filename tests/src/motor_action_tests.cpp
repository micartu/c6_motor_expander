#include "CppUTest/TestHarness.h"
#include "motor_action.h"

/// a shortcut for assiging timer's CCR register
#define CCR(tim, num) (&(tim)->CCR##num)

TEST_GROUP(MotorActionsTestGroup)
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

TEST(MotorActionsTestGroup, InitializationTest)
{
    // given
    uint32_t kReg = 123;

    // when
    tim0.CCR1 = kReg;
    tim1.CCR2 = kReg;

    // then
    LONGS_EQUAL(kReg, *(mtr.m0.CCR0));
    LONGS_EQUAL(kReg, *(mtr.m1.CCR1));
}

TEST(MotorActionsTestGroup, ActionTest)
{
    // given
    uint16_t kThrottle = 1323;
    
    // when
    apply_action_to_motor(&mtr, MLEFT, FORWARD, kThrottle);

    // then
    LONGS_EQUAL(kThrottle, *(mtr.m1.CCR0));
    LONGS_EQUAL(0, *(mtr.m1.CCR1));
}

#undef CCR