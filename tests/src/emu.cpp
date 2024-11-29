#include "emu.h"
#include <assert.h>

void emu::conduct()
{
    assert(_simulation);
    double last_measure_time = 0;
    for (double time = 0; time < _emu_time; time += _tick_time)
    {
        _simulation(_obj);
        if (time >= last_measure_time + _measure_time)
        {
            last_measure_time = time;
            measure_outputs(_obj);
        }
    }
}
