#include "CppUTest/TestHarness.h"
#include "rpm_measurer.h"

// needed headers + and implementations which control fake's behavior:
extern "C"
{
    void _set_sys_time(uint32_t time);
}

TEST_GROUP(CalculateRPMTestGroup)
{
    struct speed_measure_t rpm;

    void setup()
    {
        rpm_measurer_init (&rpm);
    }

    void teardown()
    {
    }
};

TEST(CalculateRPMTestGroup, CalculateRPMTest)
{
    // given
    rpm.pos = 30;
    calculate_rpm(&rpm);

    // when
    // then
    DOUBLES_EQUAL(30, rpm.speed, 0.1);
    LONGS_EQUAL(0, rpm.pos);
}