#include "CppUTest/TestHarness.h"
#include "pid.h"
#include "emu.h"
#include "circ_buf.h"
#include "circ_dbuf.h"

static struct cntr_pid_t pid;
static double kSetpoint = 1.0f;
static double kTickTime = 0.001f; // 1 ms
static double kSampleTime = 0.01f; // 10 ms

/// emulation of an object
/// in matlab: H = tf([10], [1 3 10]); Hd = c2d(H, 0.01) => into difference eq. =>
/// y(n) = 1.969 * y(n - 1) - 0.97 * y(n - 2) + 0.001 * (0.495 * x(n - 1) + 0.49 * x(n - 2))
void measure_outputs_object(struct controllable_t * obj)
{
    // last values from circular buffer
    double y0 = circ_double_buf_value(obj->y_last, 0);
    double y1 = circ_double_buf_value(obj->y_last, 1);
    double x0 = circ_double_buf_value(obj->x_last, 0);
    double x1 = circ_double_buf_value(obj->x_last, 1);
    // calculate next value
    obj->y = 1.969 * y0 - 0.97 * y1 + 0.001 * (0.495 * x0 + 0.49 * x1);
    // put it into circular buffer
    circ_double_buf_push(obj->y_last, obj->y);
    double x = pid_update(&pid, kSetpoint, obj->y);
    circ_double_buf_push(obj->x_last, x);
}

void simu_object_tick_events(struct controllable_t * obj)
{
}

TEST_GROUP(PIDControllerTestGroup)
{
    struct controllable_t obj;
    struct circ_buf_t ylast;
    struct circ_buf_t xlast;
    double ylastvalues[2];
    double xlastvalues[2];

    void setup()
    {
        init_controllable(&obj);
        set_controllable_measurement(&obj, measure_outputs_object);
        circ_double_buf_init(&ylast, ylastvalues, 2);
        circ_double_buf_init(&xlast, xlastvalues, 2);
        obj.y_last = &ylast;
        obj.x_last = &xlast;
    }

    void teardown()
    {
    }
};

TEST(PIDControllerTestGroup, Emu2ndOrderSystem)
{
    // given
    emu simu(&obj, simu_object_tick_events, kTickTime, kSampleTime, 1.5f /* end time */);
    // PID parameters are calculated in matlab
    // for tf => H = tf([10], [1 3 10])
    init_pid(&pid, 41.21f, 144.3f, 0.942f, kTickTime, -100, 100);
    set_sample_time(&pid, kSampleTime);

    // when
    simu.conduct();

    // then
    DOUBLES_EQUAL(kSetpoint, obj.y, 0.1);
}
