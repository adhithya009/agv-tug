#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

/*
 * Simple scalar / one-dimensional Kalman filter.
 */
typedef struct
{
    /*
     * Process noise covariance.
     *
     * Larger q means we trust the system prediction less.
     */
    float q;


    /*
     * Measurement noise covariance.
     *
     * Larger r means we trust the sensor less.
     */
    float r;


    /*
     * Current estimated value.
     */
    float x;


    /*
     * Estimate error covariance.
     */
    float p;


    /*
     * Last calculated Kalman gain.
     */
    float k;


    /*
     * Indicates whether the filter has
     * received valid initialization.
     */
    uint8_t initialized;

} Kalman1D;


/*
 * Initialize one scalar Kalman filter.
 */
void Kalman1D_Init(Kalman1D *kf,
                   float process_noise,
                   float measurement_noise,
                   float initial_estimate,
                   float initial_error);


/*
 * Feed one new sensor measurement.
 *
 * Returns updated filtered estimate.
 */
float Kalman1D_Update(Kalman1D *kf,
                      float measurement);

#endif
