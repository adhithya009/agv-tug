#include "kalman.h"


/*
 * ==============================================================
 * INITIALIZE 1D KALMAN FILTER
 * ==============================================================
 */
void Kalman1D_Init(Kalman1D *kf,
                   float process_noise,
                   float measurement_noise,
                   float initial_estimate,
                   float initial_error)
{
    if (kf == 0)
    {
        return;
    }


    /*
     * Process noise:
     *
     * How much uncertainty the model gains
     * between measurements.
     */
    kf->q = process_noise;


    /*
     * Measurement noise:
     *
     * How noisy we believe the sensor is.
     */
    kf->r = measurement_noise;


    /*
     * Initial estimated state.
     */
    kf->x = initial_estimate;


    /*
     * Initial uncertainty of estimate.
     */
    kf->p = initial_error;


    /*
     * No Kalman gain has been calculated yet.
     */
    kf->k = 0.0f;


    /*
     * Filter is ready.
     */
    kf->initialized = 1U;
}


/*
 * ==============================================================
 * UPDATE 1D KALMAN FILTER
 * ==============================================================
 */
float Kalman1D_Update(Kalman1D *kf,
                      float measurement)
{
    if (kf == 0)
    {
        /*
         * No filter structure exists.
         *
         * Safest output is unfiltered measurement.
         */
        return measurement;
    }


    /*
     * If somebody forgot to initialize the filter,
     * use the first measurement as the initial state.
     */
    if (!kf->initialized)
    {
        kf->x = measurement;
        kf->p = 1.0f;
        kf->k = 0.0f;
        kf->initialized = 1U;
    }


    /*
     * ----------------------------------------------------------
     * PREDICTION STEP
     * ----------------------------------------------------------
     *
     * For this simple scalar filter there is no
     * explicit motion model.
     *
     * State prediction:
     *
     * x(k|k-1) = x(k-1)
     *
     * Uncertainty grows by process noise:
     *
     * P = P + Q
     */
    kf->p =
        kf->p +
        kf->q;


    /*
     * ----------------------------------------------------------
     * KALMAN GAIN
     * ----------------------------------------------------------
     *
     * K = P / (P + R)
     */
    float denominator =
        kf->p +
        kf->r;


    /*
     * Avoid division by zero if invalid tuning
     * values were supplied.
     */
    if (denominator <= 0.0f)
    {
        return measurement;
    }


    kf->k =
        kf->p /
        denominator;


    /*
     * ----------------------------------------------------------
     * STATE CORRECTION
     * ----------------------------------------------------------
     *
     * x =
     * x + K(measurement - x)
     */
    kf->x =
        kf->x +
        kf->k *
        (measurement - kf->x);


    /*
     * ----------------------------------------------------------
     * COVARIANCE CORRECTION
     * ----------------------------------------------------------
     *
     * P = (1 - K)P
     */
    kf->p =
        (1.0f - kf->k) *
        kf->p;


    return kf->x;
}
