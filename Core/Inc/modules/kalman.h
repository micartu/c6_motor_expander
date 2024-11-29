#ifndef __KALMAN_H
#define __KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

struct kalman_1d_t {
    /// Measurement Uncertainty
    float err_measure;
    /// Estimation Uncertainty
    float err_estimate;
    /// Process Noise
    float q;
    /// Last Signal's Estimation
    float last_estimate;
};

void init_kalman_1d(struct kalman_1d_t * kalman);
void kalman_1d_set_measurement_error(struct kalman_1d_t * klm, float err_measure);
void kalman_1d_set_estimate_error(struct kalman_1d_t * klm, float err_est);
void kalman_1d_set_process_noise(struct kalman_1d_t * klm, float q);
float kalman_1d_update_estimate(struct kalman_1d_t * klm, float measured);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H */