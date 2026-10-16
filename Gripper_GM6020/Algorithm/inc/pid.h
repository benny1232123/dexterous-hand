#ifndef PID_H
#define PID_H

#include "filter.h"

typedef struct
{
    /* PID gains */
    float Kp;
    float Ki;
    float Kd;

    /* target value, last measured value, integral accumulator */
    float Target;
    float Last_measured;
    float integral;

    /* integral limit, output limit */
    float integralLimit;
    float outputLimit;
} PID;

/* initialise PID parameters */
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit);
/* change PID target */
void PID_SetTarget(PID *pid, float Target);
/* run one PID step */
float PID_compute(PID *pid, float measured, float dt);
/* change PID parameters (modelled PID algorithm) */
void setPID_parameter(PID *pid, float Kp, float Ki, float Kd, float integralLimit, float outputLimit);
/* reset PID state */
void ResetPID(PID *pid);

#endif
