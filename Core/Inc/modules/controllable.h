#ifndef __CONTROLLABLE_H__
#define __CONTROLLABLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stddef.h>

struct controllable_t; // declaration for func's pointer
typedef void (*func_measure_outputs_t)(struct controllable_t*);
struct circ_buf_t; // foward declaration of circular buffer

struct controllable_t
{
    /// input
    double x;
    /// output
    double y;

    /// last inputs (if needed)
    struct circ_buf_t * x_last;

    /// last outputs (if needed)
    struct circ_buf_t * y_last;

    /// function which controls how output depends on input
    func_measure_outputs_t sample_output;
};


void init_controllable(struct controllable_t * ctr);

void set_controllable_measurement(struct controllable_t * ctr, func_measure_outputs_t f);

void measure_outputs(struct controllable_t * ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLABLE_H__ */