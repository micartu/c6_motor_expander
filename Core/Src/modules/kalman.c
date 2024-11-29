#include "kalman.h"
#include <string.h>
#include <math.h>

void init_kalman_1d(struct kalman_1d_t * kalman)
{
    memset(kalman, 0, sizeof(*kalman));
}

void kalman_1d_set_measurement_error(struct kalman_1d_t * klm, float err_measure)
{
    klm->err_measure = err_measure;
}

void kalman_1d_set_estimate_error(struct kalman_1d_t * klm, float err_est)
{
    klm->err_estimate = err_est;
}

void kalman_1d_set_process_noise(struct kalman_1d_t * klm, float q)
{
    klm->q = q;
}

float kalman_1d_update_estimate(struct kalman_1d_t * klm, float measured)
{
    float kalman_gain = klm->err_estimate / (klm->err_estimate + klm->err_measure);
    float current_estimate = klm->last_estimate + kalman_gain * (measured - klm->last_estimate);
    klm->err_estimate = (1.0 - kalman_gain) * klm->err_estimate +
                        fabs(klm->last_estimate - current_estimate) * klm->q;
    klm->last_estimate = current_estimate;
    return current_estimate;
}