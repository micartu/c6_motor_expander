#include "controllable.h"
#include <string.h>
#include <assert.h>

void init_controllable(struct controllable_t * ctr)
{
    memset(ctr, 0, sizeof(*ctr));
}

void set_controllable_measurement(struct controllable_t * ctr, func_measure_outputs_t f)
{
    ctr->sample_output = f;
}

void measure_outputs(struct controllable_t * ctrl)
{
    assert(ctrl->sample_output);
    ctrl->sample_output(ctrl);
}