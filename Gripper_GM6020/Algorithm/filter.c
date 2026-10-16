#include "filter.h"

#define MOTOR_ALPHA    0.04

/*
    @brief first-order low pass filter
*/
float LowPassfilter_First(LowPassfilter *LowPassfilter, float data)
{
    float output = LowPassfilter->alpha * data + (1 - LowPassfilter->alpha) * LowPassfilter->lastData;
    LowPassfilter->lastData = output;
    return output;
}

LowPassfilter motor0_omiga_filter = { 0, MOTOR_ALPHA };
LowPassfilter motor1_omiga_filter = { 0, MOTOR_ALPHA };

/* Q=0.1 R=1.0 initial recommended values
 * Q up -> trust measurement more, faster tracking but noisier
 * R up -> trust prediction more, smoother but slower */
KalmanFilter motor0_kalman = { .x = 0, .p = 1.0f, .q = 0.1f, .r = 1.0f };
KalmanFilter motor1_kalman = { .x = 0, .p = 1.0f, .q = 0.1f, .r = 1.0f };

/*
    @brief one-dimensional Kalman filter
*/
float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    /* prediction step */
    float p_pred = kf->p + kf->q;

    /* Kalman gain */
    float k = p_pred / (p_pred + kf->r);

    /* update step */
    kf->x = kf->x + k * (measurement - kf->x);
    kf->p = (1.0f - k) * p_pred;

    return kf->x;
}
