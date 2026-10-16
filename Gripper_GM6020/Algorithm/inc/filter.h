#ifndef FILTER_H
#define FILTER_H

typedef struct
{
    float lastData;
    float alpha;
} LowPassfilter;

typedef struct
{
    float x;    /* state estimate      */
    float p;    /* estimation covariance */
    float q;    /* process noise: larger = faster tracking, noisier */
    float r;    /* measurement noise: larger = smoother, slower */
} KalmanFilter;

extern LowPassfilter motor0_omiga_filter;
extern LowPassfilter motor1_omiga_filter;

extern KalmanFilter motor0_kalman;
extern KalmanFilter motor1_kalman;

float LowPassfilter_First(LowPassfilter *LowPassfilter, float data);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif
