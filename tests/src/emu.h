#pragma once
#include "controllable.h"

class emu
{
public:
    emu(controllable_t * obj,
        func_measure_outputs_t simulation,
        double tick_time,
        double measure_time,
        double emu_time) : _obj(obj),
                           _simulation(simulation),
                           _tick_time(tick_time),
                           _measure_time(measure_time),
                           _emu_time(emu_time)
    {
    }

    void conduct();

private:
    controllable_t * _obj;
    func_measure_outputs_t _simulation;
    double _tick_time;
    double _measure_time;
    double _emu_time;
};